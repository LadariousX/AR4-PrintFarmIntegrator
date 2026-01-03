package ui

import (
	"fmt"
	"image/color"
	"integrator/core"
	"log"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/app"
	"fyne.io/fyne/v2/canvas"
	"fyne.io/fyne/v2/container"
	"fyne.io/fyne/v2/data/binding"
	"fyne.io/fyne/v2/widget"
	"github.com/sqweek/dialog"
)

func Start(appState *core.AppState, rCtrl *core.RCtrl, pCtrl *core.PCtrl, jCtrl *core.JobCtrl, onReady func()) {
	a := app.New()
	w := a.NewWindow("Print Farm Integrator")

	content := buildMainPage(appState, rCtrl, pCtrl, jCtrl)
	w.SetContent(content)

	// Let window fit content size
	w.Resize(fyne.NewSize(600, 700))

	onReady()
	w.ShowAndRun()
}

func buildMainPage(state *core.AppState, rCtrl *core.RCtrl, pCtrl *core.PCtrl, jCtrl *core.JobCtrl) fyne.CanvasObject {
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
	// PRINTER STATUS =============================================================================
	PrinterRstBtn := widget.NewButton("", func() {
		pCtrl.OctoprintReset <- struct{}{}
	})
	PrintRefreshIcon := canvas.NewText("🔄", color.White)
	PrintRefreshIcon.TextSize = 20
	PrintRefreshIcon.Alignment = fyne.TextAlignCenter

	PrinterRstBtnWI := container.NewStack(PrinterRstBtn, PrintRefreshIcon)
	PrinterRstBtnWrap := container.NewGridWrap(
		fyne.NewSize(36, 36),
		PrinterRstBtnWI,
	)

	PrinterStatus := widget.NewLabelWithData(state.OctoStatus)
	PrinterStatus.Wrapping = fyne.TextWrapWord

	PStatusBar := container.NewBorder(
		nil, nil,
		PrinterRstBtnWrap, nil,
		PrinterStatus,
	)
	// FILE SELECTION =============================================================================
	// TODO: make not writeable while in job
	// FILE SELECTION - G CODE
	gcodeTitle := widget.NewLabelWithStyle(
		"Set file to print",
		fyne.TextAlignLeading,
		fyne.TextStyle{Bold: true},
	)
	gcodeEntry := widget.NewEntry()
	gcodeEntry.SetPlaceHolder("path/to/file.gcode")

	gcodeBtn := widget.NewButton("📂", func() {
		printFilepath, err := dialog.File().Title("Open Gcode").Load()
		if err != nil {
			log.Printf("Error from printer filepicker: %v", err)
			return // Important: don't continue if there's an error
		}
		gcodeEntry.SetText(printFilepath) // update txt box to show the filepath.

		go func() {
			pCtrl.SetPrinterProg <- printFilepath
		}()
	})

	gcodeBtnWrap := container.NewGridWrap(
		fyne.NewSize(36, 36),
		gcodeBtn,
	)
	gcodeEntry.OnChanged = func(text string) {
		if text != "" && text != "/path/to/robotfile" {
			go func() {
				pCtrl.SetPrinterProg <- text
			}()
		}
	}
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
		armFilepath, err := dialog.File().Title("Open Gcode").Load()
		if err != nil {
			log.Printf("Error from arm filepicker: %v", err)
			return // Important: don't continue if there's an error
		}
		armFileEntry.SetText(armFilepath) // update txt box to show the filepath.
		rCtrl.SetProg <- armFilepath
	})

	armFileBtnWrap := container.NewGridWrap(
		fyne.NewSize(36, 36),
		armFileBtn,
	)
	armFileEntry.OnChanged = func(text string) {
		if text != "" && text != "/path/to/robotfile" {
			rCtrl.SetProg <- text
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

	// Repetitions row: +/−/count
	minusBtn := widget.NewButton("−", func() {
		jCtrl.DecrementRepeat <- struct{}{}
	})
	repeatLabel := widget.NewLabelWithData(binding.IntToString(jCtrl.RepetitionCount))
	plusBtn := widget.NewButton("+", func() {
		jCtrl.IncrementRepeat <- struct{}{}
	})

	repeatRow := container.NewHBox(
		widget.NewLabel("Repetitions:"),
		minusBtn,
		repeatLabel,
		plusBtn,
	)

	// Cycle and remaining labels
	cycleLabel := widget.NewLabel("Cycle: 0")
	remainingLabel := widget.NewLabel("Remaining: 0")

	jCtrl.CurrentCycle.AddListener(binding.NewDataListener(func() {
		current, _ := jCtrl.CurrentCycle.Get()
		cycleLabel.SetText(fmt.Sprintf("Cycle: %d", current))
	}))

	jCtrl.RemainingCycles.AddListener(binding.NewDataListener(func() {
		remaining, _ := jCtrl.RemainingCycles.Get()
		remainingLabel.SetText(fmt.Sprintf("Remaining: %d", remaining))
	}))

	cycleInfoRow := container.NewHBox(
		cycleLabel,
		widget.NewLabel("  |  "),
		remainingLabel,
	)

	// Time remaining & cycleState labels
	cycleTimeLabel := widget.NewLabelWithData(jCtrl.CycleTimeRemaining)
	jobTimeLabel := widget.NewLabelWithData(jCtrl.JobTimeRemaining)
	jobStageLabel := widget.NewLabelWithData(jCtrl.CycleStage)

	timeRow := container.NewHBox(
		widget.NewLabel("Cycle Time:"),
		cycleTimeLabel,
		widget.NewLabel("  |  Job Time:"),
		jobTimeLabel,
		widget.NewLabel("  |  Cycle Stage:"),
		jobStageLabel,
	)

	// Progress bar
	progressBar := widget.NewProgressBarWithData(jCtrl.Progress)

	// Job button
	startBtn := widget.NewButton("Start Job", func() {
		jCtrl.StartJob <- struct{}{}
	})
	startBtn.Importance = widget.HighImportance

	// Pause / Resume button (toggles)
	pauseBtn := widget.NewButton("Pause Job", func() {
		jCtrl.PauseJob <- struct{}{}
	})

	// Update button text when pause state changes
	jCtrl.IsPaused.AddListener(binding.NewDataListener(func() {
		isPaused, _ := jCtrl.IsPaused.Get()
		if isPaused {
			pauseBtn.SetText("Resume Job")
		} else {
			pauseBtn.SetText("Pause Job")
		}
	}))

	// Stop button
	stopBtn := widget.NewButton("Stop Job", func() {
		jCtrl.StopJob <- struct{}{}
	})
	stopBtn.Importance = widget.DangerImportance

	controlRow := container.NewHBox(
		startBtn,
		pauseBtn,
		stopBtn,
	)

	// LOG OUTPUT BOX =============================================================================
	logTitle := widget.NewLabelWithStyle(
		"Output Log",
		fyne.TextAlignLeading,
		fyne.TextStyle{Bold: true},
	)

	clearLogBtn := widget.NewButton("Clear Log", func() {
		jCtrl.LogWriter.Clear()
	})
	clearLogBtn.Importance = widget.LowImportance

	autoScrollCheck := widget.NewCheck("Auto-scroll", nil)
	autoScrollCheck.Checked = true

	logTitleRow := container.NewBorder(
		nil, nil,
		logTitle, container.NewHBox(autoScrollCheck, clearLogBtn),
	)

	// Use RichText with explicit text color for better contrast
	logText := widget.NewRichTextFromMarkdown("")
	logText.Wrapping = fyne.TextWrapWord

	logScroll := container.NewScroll(logText)
	logScroll.SetMinSize(fyne.NewSize(0, 150))

	// Update log text when binding changes
	jCtrl.LogOutput.AddListener(binding.NewDataListener(func() {
		text, _ := jCtrl.LogOutput.Get()
		logText.ParseMarkdown(text)

		// Auto-scroll to bottom if enabled
		if autoScrollCheck.Checked {
			logScroll.ScrollToBottom()
		}
	}))

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
		cycleInfoRow,
		timeRow,
		progressBar,
		controlRow,
		widget.NewSeparator(),

		logTitleRow,
		logScroll,
	)

	return container.NewVScroll(container.NewPadded(page))
}
