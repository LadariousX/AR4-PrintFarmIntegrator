package controllers

import (
	"PrintFarmIntegrator/core"
	"context"
	"fmt"
	"log"
	"path/filepath"
	"time"
)

// OctoPrintController is the main controller goroutine for managing OctoPrint
func OctoPrintController(state *core.AppState, ctx context.Context, pCtrl *core.PCtrl) {
	// Connect to OctoPrint
	octo := NewOctoPrint()
	ticker := time.NewTicker(5 * time.Second)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done(): // app shutting down
			return

		case file := <-pCtrl.StartPrint:

			// better to re upload or check? how does this code even work

			// Upload and start print
			log.Printf("octoprint: Uploading and starting print: %s", file)

			// Upload file first
			if err := octo.UploadFile(file); err != nil {
				log.Printf("octoprint: Upload error: %v", err)
				break
			}

			// Start the print4
			filename := filepath.Base(file)
			if err := octo.StartPrint(filename); err != nil {
				log.Printf("octoprint: Start print error: %v", err)
			}

		case <-pCtrl.PausePrint:
			log.Println("octoprint: Pausing print...")
			if err := octo.PausePrint(); err != nil {
				log.Printf("octoprint: Error pausing: %v", err)
			}

		case <-pCtrl.ResumePrint:
			log.Println("octoprint: Resuming print...")
			if err := octo.ResumePrint(); err != nil {
				log.Printf("octoprint: Error resuming: %v", err)
			}

		case <-pCtrl.StopPrint:
			log.Println("octoprint: Stopping print...")
			if err := octo.StopPrint(); err != nil {
				log.Printf("octoprint: Error stopping: %v", err)
			}

		case <-ticker.C:

			PStatus, PStatusErr := octo.GetPrinterResp()

			if PStatusErr != nil {
				log.Printf("octoprint: Error getting printer status: \n%v", PStatusErr)
				state.PrinterStatus.Set("octoprint: Error contacting octoprint")
			} else {
				fmt.Println(PStatus.State.Text)
				state.PrinterStatus.Set(PStatus.State.Text)
			}
		}
	}
}
