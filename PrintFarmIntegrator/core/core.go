package core

import "fyne.io/fyne/v2/data/binding"

type AppState struct {
	ArmStatus     binding.String
	PrinterStatus binding.String
	CycleStatus   binding.String
}

type RCtrl struct {
	AR4CtrlReset chan struct{}
	Start        chan struct{}
	Pause        chan struct{}
	Stop         chan struct{}
	SetRobotProg chan string
}

type Position struct {
	X *float64
	Y *float64
	Z *float64
}

type PCtrl struct {
	StartPrint             chan string
	PausePrint             chan struct{}
	ResumePrint            chan struct{}
	StopPrint              chan struct{}
	PrinterStatus          chan PrinterResponse
	JobStatus              chan JobResponse
	MovePrinter            chan Position
	DisablePrinterSteppers chan struct{}
	EnablePrinterSteppers  chan struct{}
}

type JobResponse struct {
	Job struct {
		File struct {
			Name   string `json:"name"`
			Origin string `json:"origin"`
			Size   int64  `json:"size"`
			Date   int64  `json:"date"`
		} `json:"file"`
		EstimatedPrintTime float64 `json:"estimatedPrintTime"`
		Filament           map[string]struct {
			Length float64 `json:"length"`
			Volume float64 `json:"volume"`
		} `json:"filament"`
	} `json:"job"`
	Progress struct {
		Completion    float64 `json:"completion"`
		Filepos       int64   `json:"filepos"`
		PrintTime     int     `json:"printTime"`
		PrintTimeLeft int     `json:"printTimeLeft"`
	} `json:"progress"`
	State string `json:"state"`
}

type PrinterResponse struct {
	State struct {
		Text  string `json:"text"`
		Flags struct {
			Operational   bool `json:"operational"`
			Paused        bool `json:"paused"`
			Printing      bool `json:"printing"`
			Cancelling    bool `json:"cancelling"`
			Pausing       bool `json:"pausing"`
			SdReady       bool `json:"sdReady"`
			Error         bool `json:"error"`
			Ready         bool `json:"ready"`
			ClosedOrError bool `json:"closedOrError"`
		} `json:"flags"`
	} `json:"state"`
}
