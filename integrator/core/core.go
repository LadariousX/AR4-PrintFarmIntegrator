package core

import "fyne.io/fyne/v2/data/binding"

type AppState struct {
	ArmStatus   binding.String
	OctoStatus  binding.String
	PrintStatus binding.Untyped
	CycleStatus binding.String
}

type RCtrl struct {
	AR4CtrlReset chan struct{}
	Start        chan struct{}
	Stop         chan struct{}
	Load         chan struct{}
	SetProg      chan string
	SetProgIndex chan int
	State        chan RobotState
}

type RobotState struct {
	State string
	Line  int
}

type Position struct {
	X *float64
	Y *float64
	Z *float64
}

type PCtrl struct {
	OctoprintReset         chan struct{}
	StartPrint             chan struct{}
	PausePrint             chan struct{}
	ResumePrint            chan struct{}
	StopPrint              chan struct{}
	EnablePrinterSteppers  chan struct{}
	DisablePrinterSteppers chan struct{}
	UploadFile             chan struct{}

	SetPrinterProg chan string
	PrinterStatus  chan PrinterResponse
	JobStatus      chan JobResponse
	MovePrinter    chan Position
}

type JobCtrl struct {
	StartJob        chan struct{}
	PauseJob        chan struct{}
	StopJob         chan struct{}
	IncrementRepeat chan struct{}
	DecrementRepeat chan struct{}
	IsPaused        binding.Bool
	LogOutput       binding.String
	LogWriter       *LogWriter

	RepetitionCount    binding.Int
	CurrentCycle       binding.Int
	RemainingCycles    binding.Int
	CycleTimeRemaining binding.String
	JobTimeRemaining   binding.String
	CycleStage         binding.String
	Progress           binding.Float
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
