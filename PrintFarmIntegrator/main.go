package main

import (
	"PrintFarmIntegrator/controllers"
	"PrintFarmIntegrator/core"
	"PrintFarmIntegrator/ui"
	"context"
	"log"

	"fyne.io/fyne/v2/data/binding"
	"github.com/joho/godotenv"
)

const envPath = "/Users/layden/PycharmProjects/ar4-hmi/.env"

func main() {
	log.Println("Application starting")

	envErr := godotenv.Load(envPath)
	if envErr != nil {
		log.Fatalf("Error loading .env file: %v", envErr)
	}

	appState := &core.AppState{
		ArmStatus:     binding.NewString(),
		PrinterStatus: binding.NewString(),
		CycleStatus:   binding.NewString()}

	rCtrl := &core.RCtrl{
		AR4CtrlReset: make(chan struct{}),
		SetRobotProg: make(chan string, 1),

		Start: make(chan struct{}),
		Pause: make(chan struct{}),
		Stop:  make(chan struct{})}

	pCtrl := &core.PCtrl{
		StartPrint:  make(chan string),
		PausePrint:  make(chan struct{}),
		ResumePrint: make(chan struct{}),
		StopPrint:   make(chan struct{}),
		//PrinterStatus: make(chan struct{}),
		//PrinterStatusResponse:  make(chan map[string]interface{}),
		MovePrinter:            make(chan core.Position),
		DisablePrinterSteppers: make(chan struct{}),
		EnablePrinterSteppers:  make(chan struct{})}

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	ui.Start(appState, rCtrl, func() {
		go controllers.AR4Controller(appState, rCtrl)
		go controllers.OctoPrintController(appState, ctx, pCtrl)
	})

	//pCtrl.StartPrint <- "/Users/layden/PycharmProjects/ar4-hmi/TestUpload.gcode"
}
