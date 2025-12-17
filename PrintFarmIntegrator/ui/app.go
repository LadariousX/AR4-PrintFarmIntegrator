package ui

import (
	"PrintFarmIntegrator/core"
	"image/color"
	"log"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/app"
	"fyne.io/fyne/v2/canvas"
	"fyne.io/fyne/v2/container"
	"fyne.io/fyne/v2/widget"
	"github.com/sqweek/dialog"
)

func Start(state *core.AppState, control *core.RCtrl, onReady func()) {
	a := app.New()
	w := a.NewWindow("Print Farm Integrator")

	w.SetContent(buildMainPage(state, control))
	w.Resize(fyne.NewSize(760, 520))
	onReady()
	w.ShowAndRun()
}

func buildMainPage(state *core.AppState, rCtrl *core.RCtrl) fyne.CanvasObject {
	// PRINTER STATUS =============================================================================
	PrinterRstBtn := widget.NewButton("", func() {
		// preset <- struct{}{}
	})
	PrintRefreshIcon := canvas.NewText("🔄", color.White)
	PrintRefreshIcon.TextSize = 20
	PrintRefreshIcon.Alignment = fyne.TextAlignCenter

	PrinterRstBtnWI := container.NewStack(PrinterRstBtn, PrintRefreshIcon)
	PrinterRstBtnWrap := container.NewGridWrap(
		fyne.NewSize(36, 36),
		PrinterRstBtnWI,
	)

	PrinterStatus := widget.NewLabelWithData(state.PrinterStatus)
	PrinterStatus.Wrapping = fyne.TextWrapWord

	PStatusBar := container.NewBorder(
		nil, nil,
		PrinterRstBtnWrap, nil,
		PrinterStatus,
	)

	// arm STATUS BAR =============================================================================
	AR4CtrlRstBtn := widget.NewButton("", func() { rCtrl.AR4CtrlReset <- struct{}{} })
	refreshIcon := canvas.NewText("🔄", color.White)
	refreshIcon.TextSize = 20
	refreshIcon.Alignment = fyne.TextAlignCenter

	AR4CtrlRstBtnWI := container.NewStack(AR4CtrlRstBtn, refreshIcon)
	AR4CtrlRstBtnWrap := container.NewGridWrap(
		fyne.NewSize(36, 36),
		AR4CtrlRstBtnWI,
	)

	armStatus := widget.NewLabelWithData(state.ArmStatus)
	armStatus.Wrapping = fyne.TextWrapWord

	AStatusBar := container.NewBorder(
		nil, nil,
		AR4CtrlRstBtnWrap, nil,
		armStatus,
	)
	// FILE SELECTION =============================================================================
	// FILE SELECTION - G CODE
	gcodeTitle := widget.NewLabelWithStyle(
		"Set file to print",
		fyne.TextAlignLeading,
		fyne.TextStyle{Bold: true},
	)

	gcodeBtn := widget.NewButton("📂", func() {})
	gcodeBtnWrap := container.NewGridWrap(
		fyne.NewSize(36, 36),
		gcodeBtn,
	)

	gcodeEntry := widget.NewEntry()
	gcodeEntry.SetPlaceHolder("path/to/file.gcode")

	gcodeRow := container.NewBorder(
		nil, nil,
		gcodeBtnWrap, nil,
		gcodeEntry,
	)

	// FILE SELECTION — ARM

	armFileTitle := widget.NewLabelWithStyle(
		"Set arm file",
		fyne.TextAlignLeading,
		fyne.TextStyle{Bold: true},
	)

	armFileEntry := widget.NewEntry()
	armFileEntry.SetPlaceHolder("/path/to/robotfile")

	armFileBtn := widget.NewButton("📂", func() {
		armfilepath, err := dialog.File().Title("Open Gcode").Load()
		if err != nil {
			log.Printf("Error from arm filepicker: %v", err)
			return // Important: don't continue if there's an error
		}

		// Update the entry widget to show the selected file
		armFileEntry.SetText(armfilepath)

		// Send to robot channel
		rCtrl.SetRobotProg <- armfilepath
	})

	armFileBtnWrap := container.NewGridWrap(
		fyne.NewSize(36, 36),
		armFileBtn,
	)

	armFileEntry.OnChanged = func(text string) {
		if text != "" && text != "/path/to/robotfile" {
			rCtrl.SetRobotProg <- text
		}
	}

	armFileRow := container.NewBorder(
		nil, nil,
		armFileBtnWrap, nil,
		armFileEntry,
	)

	// JOB SETTINGS =============================================================================
	jobTitle := widget.NewLabelWithStyle(
		"Job settings",
		fyne.TextAlignLeading,
		fyne.TextStyle{Bold: true},
	)

	minusBtn := widget.NewButton("−", func() {})
	repeatLabel := widget.NewLabel("1")
	plusBtn := widget.NewButton("+", func() {})
	progressLabel := widget.NewLabel("Cycle: 0   Remaining: 0")

	repeatRow := container.NewHBox(
		widget.NewLabel("Repetitions"),
		minusBtn,
		repeatLabel,
		plusBtn,
		progressLabel,
	)

	startBtn := widget.NewButton("Start Job", func() {})
	startBtn.Importance = widget.HighImportance

	pauseBtn := widget.NewButton("Pause All", func() {})
	stopBtn := widget.NewButton("Stop All", func() {})
	stopBtn.Importance = widget.DangerImportance

	controlRow := container.NewHBox(
		startBtn,
		pauseBtn,
		stopBtn,
	)

	// PAGE LAYOUT (SCROLLABLE) =============================================================================
	page := container.NewVBox(
		AStatusBar,
		widget.NewSeparator(),

		PStatusBar,
		widget.NewSeparator(),

		gcodeTitle,
		gcodeRow,

		armFileTitle,
		armFileRow,
		widget.NewSeparator(),

		jobTitle,
		repeatRow,
		progressLabel,
		controlRow,
	)

	return container.NewVScroll(container.NewPadded(page))
}
