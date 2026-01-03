package controllers

import (
	"integrator/core"
	"log"
	"time"
)

func JobControl(state *core.AppState, jCtrl *core.JobCtrl, rCtrl *core.RCtrl, pCtrl *core.PCtrl) {
	ticker := time.NewTicker(100 * time.Millisecond)
	defer ticker.Stop()

	repetitions := 1
	currentCycle := 0
	isJobRunning := false
	isJobPaused := false

	// Job stage tracking
	type JobStage int
	const (
		StageIdle JobStage = iota
		StagePrinting
		StageCooling
		StageRobotWorking
	)
	currentStage := StageIdle
	var coolingStartTime time.Time

	// Track previous states to detect transitions
	var prevPrinterState string
	var prevRobotState string

	// Initialize bindings
	_ = jCtrl.RepetitionCount.Set(repetitions)
	_ = jCtrl.CurrentCycle.Set(currentCycle)
	_ = jCtrl.RemainingCycles.Set(repetitions)
	_ = jCtrl.CycleTimeRemaining.Set("--:--")
	_ = jCtrl.JobTimeRemaining.Set("--:--")
	_ = jCtrl.Progress.Set(0.0)
	_ = jCtrl.IsPaused.Set(false)

	// Track latest states from channels
	var currentRobotState core.RobotState
	var currentPrinterJobState core.JobResponse

	for {
		select {
		case robotState := <-rCtrl.State:
			// Track state transition
			prevRobotState = currentRobotState.State
			currentRobotState = robotState

			if robotState.State == "estop" {
				log.Println("E-STOP pressed: Stopping job and resetting printer...")
				pCtrl.OctoprintReset <- struct{}{}
				pCtrl.StopPrint <- struct{}{}
				jCtrl.StopJob <- struct{}{}
				log.Println("Reset system to continue")
			}

		case printerJobState := <-pCtrl.JobStatus:
			// Track state transition
			prevPrinterState = currentPrinterJobState.State
			currentPrinterJobState = printerJobState

		case <-jCtrl.StartJob:
			if !isJobRunning {
				log.Println("Starting job...")
				isJobRunning = true
				isJobPaused = false
				currentCycle = 0
				jCtrl.CurrentCycle.Set(currentCycle)
				jCtrl.IsPaused.Set(false)
				jCtrl.Progress.Set(0.0)

			} else {
				log.Println("Job already running")
			}

		case <-jCtrl.PauseJob: // toggle pause/resume
			if isJobRunning && !isJobPaused {
				log.Println("Pausing job...")
				isJobPaused = true
				jCtrl.IsPaused.Set(true)
				rCtrl.Stop <- struct{}{}
				pCtrl.PausePrint <- struct{}{}

			} else if isJobRunning && isJobPaused {
				log.Println("Resuming job...")
				isJobPaused = false
				jCtrl.IsPaused.Set(false)

				// TODO: safely resume both systems
				// safe take off. check positions and who was moving
			}

		case <-jCtrl.StopJob:
			//if isJobRunning { // decided to force a job to stop.
			pCtrl.StopPrint <- struct{}{}
			rCtrl.Stop <- struct{}{}

			log.Println("Stopping job...")
			isJobRunning = false
			isJobPaused = false
			currentCycle = 0
			jCtrl.CurrentCycle.Set(currentCycle)
			jCtrl.RemainingCycles.Set(repetitions)
			jCtrl.IsPaused.Set(false)
			jCtrl.Progress.Set(0.0)
			jCtrl.CycleTimeRemaining.Set("--:--")
			jCtrl.JobTimeRemaining.Set("--:--")
			//}

		case <-jCtrl.IncrementRepeat:
			//if !isJobRunning {
			repetitions++
			jCtrl.RepetitionCount.Set(repetitions)
			jCtrl.RemainingCycles.Set(repetitions - currentCycle)
			//}

		case <-jCtrl.DecrementRepeat:
			//if !isJobRunning && repetitions > 1 {
			if repetitions > 1 {
				repetitions--
				jCtrl.RepetitionCount.Set(repetitions)
				jCtrl.RemainingCycles.Set(repetitions - currentCycle)
			}

		case <-ticker.C: // job progress control flow
			if isJobRunning && !isJobPaused {
				if (repetitions - currentCycle) > 0 {
					// State machine for job cycle management
					switch currentStage {

					case StageIdle: // Start the print cycle
						_ = jCtrl.CycleStage.Set("Idle")
						log.Printf("Starting cycle %d/%d - Starting print", currentCycle+1, repetitions)
						pCtrl.StartPrint <- struct{}{}
						currentStage = StagePrinting

					case StagePrinting: // Wait for printer to complete: Operational -> Printing -> Operational
						_ = jCtrl.CycleStage.Set("Printing")

						// Detect transition TO Operational (from Printing state)
						if prevPrinterState == "Printing" && currentPrinterJobState.State == "Operational" {
							log.Println("Print complete, starting cooling period...")
							coolingStartTime = time.Now()
							currentStage = StageCooling
						}

					case StageCooling: // Wait 2 minutes for part to cool
						_ = jCtrl.CycleStage.Set("Cooling Print")
						if time.Since(coolingStartTime) >= 2*time.Second {
							//if time.Since(coolingStartTime) >= 2*time.Minute {
							log.Println("Cooling complete, starting robot operation...")
							rCtrl.Load <- struct{}{}
							rCtrl.Start <- struct{}{}
							currentStage = StageRobotWorking
						}

					case StageRobotWorking: // Wait for robot to complete: Operational -> running -> Operational
						// Detect transition TO Operational (from running state)
						_ = jCtrl.CycleStage.Set("Robot Working")

						if prevRobotState == "running" && currentRobotState.State == "Operational" {
							log.Printf("Robot operation complete, cycle %d finished", currentCycle+1)

							// Increment cycle counters
							currentCycle++
							jCtrl.CurrentCycle.Set(currentCycle)
							jCtrl.RemainingCycles.Set(repetitions - currentCycle)

							// Update progress
							progress := float64(currentCycle) / float64(repetitions)
							jCtrl.Progress.Set(progress)

							// Return to idle to start the next cycle
							currentStage = StageIdle
						}
					}

				} else {
					// Job complete
					log.Println("Job completed all cycles")
					isJobRunning = false
					currentStage = StageIdle
					jCtrl.Progress.Set(1.0)
				}
			}
		}
	}
}
