package controllers

import (
	"context"
	"integrator/core"
	"log"
	"path/filepath"
	"time"
)

// OctoPrintController is the main controller goroutine for managing OctoPrint
func OctoPrintController(appState *core.AppState, ctx context.Context, pCtrl *core.PCtrl) {
	// Launch and connect to OctoPrint
	err := StartOctoPrintServ(appState, false)
	if err != nil {
		log.Printf("octoprint: Error starting OctoPrint: %v", err)
		appState.OctoStatus.Set("octoprint: Error starting OctoPrint")
		return
	}
	octo := NewOctoPrint()
	ticker := time.NewTicker(5 * time.Second)
	defer ticker.Stop()
	var file string

	for {
		select {
		case <-ctx.Done(): // app shutting down
			return

		case <-pCtrl.OctoprintReset:
			_ = StartOctoPrintServ(appState, true)

		case <-pCtrl.StartPrint:
			if file == "" {
				log.Println("octoprint: No file selected")
				break
			}
			// upload & start
			if err := octo.UploadFile(file); err != nil {
				log.Print(err)
			}
			filename := filepath.Base(file)
			if err := octo.StartPrint(filename); err != nil {
				log.Print(err)
			}

		case <-pCtrl.PausePrint:
			if err := octo.PausePrint(); err != nil {
				log.Print(err)
			}

		case <-pCtrl.ResumePrint:
			if err := octo.ResumePrint(); err != nil {
				log.Print(err)
			}

		case <-pCtrl.StopPrint:
			if err := octo.StopPrint(); err != nil {
				log.Print(err)
			}

		case file = <-pCtrl.SetPrinterProg:
			// the system reuploads every time pCtrl.StartPrint

		case <-ticker.C:
			PStatus, PStatusErr := octo.GetPrinterResp()
			JStatus, JStatusErr := octo.GetJobResp()
			//send Printer and job status through channels
			if PStatusErr != nil || JStatusErr != nil {
				log.Print(PStatusErr, JStatusErr)
				appState.OctoStatus.Set("Octoprint status: Unable to fetch status(s), see log")
				continue
			}
			appState.OctoStatus.Set("Octoprint status: " + PStatus.State.Text)
			appState.PrintStatus.Set(JStatus)

			// Send status through channels
			select {
			case pCtrl.PrinterStatus <- *PStatus:
			default:
			}
			select {
			case pCtrl.JobStatus <- *JStatus:
			default:
			}
		}
	}
}
