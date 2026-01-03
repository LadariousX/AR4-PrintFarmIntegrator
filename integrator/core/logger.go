package core

import (
	"os"
	"strings"
	"sync"

	"fyne.io/fyne/v2/data/binding"
)

type LogWriter struct {
	buffer []string
	output binding.String
	mu     sync.Mutex
}

func NewLogWriter(output binding.String) *LogWriter {
	return &LogWriter{
		buffer: make([]string, 0, 100),
		output: output,
	}
}

func (lw *LogWriter) Write(p []byte) (n int, err error) {
	lw.mu.Lock()
	defer lw.mu.Unlock()

	message := string(p)
	lw.buffer = append(lw.buffer, message)

	// Keep last 100 lines
	if len(lw.buffer) > 100 {
		lw.buffer = lw.buffer[1:]
	}

	lw.output.Set(strings.Join(lw.buffer, "\n"))

	// Also write to stdout
	return os.Stdout.Write(p)
}

func (lw *LogWriter) Clear() {
	lw.mu.Lock()
	defer lw.mu.Unlock()

	lw.buffer = make([]string, 0, 100)
	lw.output.Set("")
}
