package main

import (
	"context"
	"integrator/controllers"
	"integrator/core"
	"integrator/ui"
	"log"
	"time"

	"fyne.io/fyne/v2/data/binding"
	"github.com/joho/godotenv"
)

const envPath = "/Users/layden/PycharmProjects/AR4-Integrator/.env"

func main() {
	log.Println("Application starting")

	envErr := godotenv.Load(envPath)
	if envErr != nil {
		log.Fatalf("Error loading .env file: %v", envErr)
	}

	appState := &core.AppState{
		ArmStatus:   binding.NewString(),
		OctoStatus:  binding.NewString(),
		PrintStatus: binding.NewUntyped(),
		CycleStatus: binding.NewString()}

	rCtrl := &core.RCtrl{
		AR4CtrlReset: make(chan struct{}, 1),
		Start:        make(chan struct{}, 1),
		Stop:         make(chan struct{}, 1),
		Load:         make(chan struct{}, 1),
		SetProg:      make(chan string, 1),
		SetProgIndex: make(chan int, 1),
		State:        make(chan core.RobotState, 1)}

	pCtrl := &core.PCtrl{
		OctoprintReset:         make(chan struct{}, 1),
		StartPrint:             make(chan struct{}, 1),
		PausePrint:             make(chan struct{}, 1),
		ResumePrint:            make(chan struct{}, 1),
		StopPrint:              make(chan struct{}, 1),
		EnablePrinterSteppers:  make(chan struct{}, 1),
		DisablePrinterSteppers: make(chan struct{}, 1),
		UploadFile:             make(chan struct{}, 1),
		SetPrinterProg:         make(chan string, 1),
		MovePrinter:            make(chan core.Position, 1),
		JobStatus:              make(chan core.JobResponse, 10),
		PrinterStatus:          make(chan core.PrinterResponse, 10)}

	// Create log output binding first
	logOutput := binding.NewString()
	logWriter := core.NewLogWriter(logOutput)

	jCtrl := &core.JobCtrl{
		StartJob:           make(chan struct{}, 1),
		PauseJob:           make(chan struct{}, 1),
		StopJob:            make(chan struct{}, 1),
		IncrementRepeat:    make(chan struct{}, 1),
		DecrementRepeat:    make(chan struct{}, 1),
		RepetitionCount:    binding.NewInt(),
		CurrentCycle:       binding.NewInt(),
		RemainingCycles:    binding.NewInt(),
		CycleTimeRemaining: binding.NewString(),
		JobTimeRemaining:   binding.NewString(),
		Progress:           binding.NewFloat(),
		IsPaused:           binding.NewBool(),
		CycleStage:         binding.NewString(),
		LogOutput:          logOutput,
		LogWriter:          logWriter}

	// Set up logging to write to UI log box
	log.SetOutput(logWriter)

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	ui.Start(appState, rCtrl, pCtrl, jCtrl, func() {
		go controllers.JobControl(appState, jCtrl, rCtrl, pCtrl)
		go controllers.AR4Controller(appState, rCtrl)
		go controllers.OctoPrintController(appState, ctx, pCtrl)

		time.Sleep(10 * time.Second)
	})

}
