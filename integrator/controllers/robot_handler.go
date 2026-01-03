package controllers

import (
	"fmt"
	"integrator/core"
	"log"
	"net"
	"time"
)

func AR4Controller(appState *core.AppState, rCtrl *core.RCtrl) {
	var conn net.Conn
	var err error
	var reset = false
	var robotProg = ""

	ticker := time.NewTicker(2 * time.Second)
	defer ticker.Stop()

	// TODO: reopen prog if robot conn fails

	// TODO: make sure error labels only happen in the highest function / where printed
	for { // main robot loop & go routine
		if conn == nil {
			appState.ArmStatus.Set("Arm status: connecting")
			conn, err = StartAR4Ctrl(appState, reset)
			reset = false
			if err != nil || conn == nil {
				log.Println("AR4 connection failed: ", err)
				appState.ArmStatus.Set("Arm status: disconnected")
				time.Sleep(2 * time.Second)
				continue
			}
			if _, err := calibrate(conn); err != nil {
				log.Printf("robot: Failed to calibrate: %v", err)
				appState.ArmStatus.Set("Arm error: calibration failed")
			}
			log.Println("AR4 connected")
			appState.ArmStatus.Set("Arm status: connected")
		}

		select {
		case <-rCtrl.AR4CtrlReset:
			log.Println("AR4 reset requested")
			appState.ArmStatus.Set("Arm status: resetting")
			conn = closeConn(conn)
			reset = true

		case <-rCtrl.Start:
			// TODO: implement error channel to pause progress
			if resp, err := runProg(conn); err != nil {
				log.Printf("robot: Failed to start program: %v", err)
				appState.ArmStatus.Set("Arm status: error starting")
			} else if ok, _ := resp["ok"].(bool); ok {
				log.Println("robot: Program started")
				appState.ArmStatus.Set("Arm status: running")
			}

		case <-rCtrl.Stop:
			log.Println("robot: Stop requested")
			if resp, err := stopProg(conn); err != nil {
				log.Printf("robot: Failed to stop: %v", err)
				appState.ArmStatus.Set("Arm status: error stopping")
			} else if ok, _ := resp["ok"].(bool); ok {
				log.Println("robot: Stopped")
				appState.ArmStatus.Set("Arm status: stopped")
			}

		case path := <-rCtrl.SetProg:
			//log.Printf("robot: set program: %s", path)
			robotProg = path

		case <-rCtrl.Load:
			if robotProg != "" {
				if resp, err := loadProg(conn, robotProg); err != nil {
					log.Printf("robot: Failed to load program: %v", err)
					appState.ArmStatus.Set("Arm error: loading program see log")
				} else if ok, _ := resp["ok"].(bool); ok {
					log.Println("robot: Program loaded")
					appState.ArmStatus.Set("Arm status: program loaded")
				}
			}

		case index := <-rCtrl.SetProgIndex:
			log.Printf("robot: Setting program index: %d", index)
			if resp, err := setIndex(conn, index); err != nil {
				log.Printf("robot: Failed to set index: %v", err)
				appState.ArmStatus.Set("Arm status: error setting index")
			} else if ok, _ := resp["ok"].(bool); ok {
				log.Printf("robot: Index set to %d", index)
				appState.ArmStatus.Set(fmt.Sprintf("Arm status: index set to %d", index))
			}

		case <-ticker.C:
			// Periodic status update
			status, err := getStatus(conn)
			if err != nil {
				// Connection lost
				if netErr, ok := err.(net.Error); ok && netErr.Timeout() {
					log.Println("AR4Ctrl status timeout")
				} else {
					log.Printf("AR4Ctrl TCP lost: %v", err)
					appState.ArmStatus.Set("Arm status: disconnected")
					conn = closeConn(conn)
					continue
				}
			}

			if status != nil {
				// Update app state with robot status
				state := status["state"]
				line := status["line"]

				var statusStr string
				var robotState core.RobotState

				if state != nil {
					stateStr := state.(string)
					robotState.State = stateStr

					// Convert line to int if present
					if line != nil {
						if lineFloat, ok := line.(float64); ok {
							robotState.Line = int(lineFloat)
						}
					}

					switch stateStr {
					case "estop":
						statusStr = "Arm status: E-STOP"
					case "running":
						statusStr = "Arm status: running"
						if line != nil {
							statusStr += fmt.Sprintf(" (line %v)", line)
						}
					case "Operational":
						statusStr = "Arm status: Operational"
					default:
						statusStr = fmt.Sprintf("Arm status: %v", state)
					}
				}

				if statusStr != "" {
					appState.ArmStatus.Set(statusStr)
				}

				// Push state through rCtrl channel (non-blocking)
				select {
				case rCtrl.State <- robotState:
				default:
					// Don't block if channel is full
				}
			}
		}
	}
}
