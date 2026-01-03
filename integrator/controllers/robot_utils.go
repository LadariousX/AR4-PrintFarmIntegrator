package controllers

import (
	"encoding/json"
	"fmt"
	"integrator/core"
	"log"
	"net"
	"os"
	"os/exec"
	"strings"
	"time"
)

func StartAR4Ctrl(appState *core.AppState, reset bool) (net.Conn, error) {
	if reset {
		fmt.Println("restarting AR4Ctrl software")
		appState.ArmStatus.Set("Arm status: Restarting AR4 Control")
	}
	appState.ArmStatus.Set("Arm status: launching AR4 Control")
	pythonPath, scriptPath := os.Getenv("PYTHON_PATH"), os.Getenv("SCRIPT_PATH")
	out, _ := exec.Command("pgrep", "-f", "AR4.py").Output()
	instances := strings.Split(string(out), "\n")
	//fmt.Println(len(instances) - 1)

	if (len(instances)-1) != 1 || reset {
		if (len(instances)-1) > 1 || reset {
			for _, instance := range instances {
				_ = exec.Command("kill", "-9", instance).Run()
			}
		}
		cmd := exec.Command(pythonPath, scriptPath)
		cmd.Stdout = os.Stdout
		cmd.Stderr = os.Stderr
		if err := cmd.Start(); err != nil {
			return nil, fmt.Errorf("error launching new AR4Control instance, %w", err)
		}
		log.Println("started new AR4Control instance")

	}

	// connect tcp
	return connectTCP()

}

func connectTCP() (net.Conn, error) { // Reconnect TCP with retry until timeout
	deadline := time.Now().Add(15 * time.Second)
	for time.Now().Before(deadline) {
		conn, err := net.DialTimeout("tcp", "127.0.0.1:5555", 500*time.Millisecond)
		if err == nil {
			return conn, nil
		}
		time.Sleep(500 * time.Millisecond)
	}

	return nil, fmt.Errorf("AR4 TCP reconnect timeout")
}

func closeConn(conn net.Conn) net.Conn {
	if conn != nil {
		_ = conn.Close()
	}
	return nil
}

// sendTCPCommand sends a JSON command to the AR4 robot and returns the response
func sendTCPCommand(conn net.Conn, command map[string]interface{}) (map[string]interface{}, error) {
	// Encode command to JSON
	cmdBytes, err := json.Marshal(command)
	if err != nil {
		return nil, fmt.Errorf("failed to marshal command: %w", err)
	}

	cmdBytes = append(cmdBytes, '\n')
	if _, err := conn.Write(cmdBytes); err != nil {
		return nil, fmt.Errorf("failed to send command: %w", err)
	}

	// Read response with timeout
	_ = conn.SetReadDeadline(time.Now().Add(2 * time.Second))
	buf := make([]byte, 4096)
	n, err := conn.Read(buf)
	if err != nil {
		return nil, fmt.Errorf("failed to read response: %w", err)
	}

	// Parse JSON response
	var response map[string]interface{}
	if err := json.Unmarshal(buf[:n], &response); err != nil {
		return nil, fmt.Errorf("failed to parse response: %w", err)
	}

	return response, nil
}

// getStatus requests current robot status
func getStatus(conn net.Conn) (map[string]interface{}, error) {
	return sendTCPCommand(conn, map[string]interface{}{"cmd": "status"})
}

// loadProg loads a program file
func loadProg(conn net.Conn, filePath string) (map[string]interface{}, error) {
	return sendTCPCommand(conn, map[string]interface{}{
		"cmd":  "load_prog",
		"path": filePath,
	})
}

// runProg starts or resumes the program
func runProg(conn net.Conn) (map[string]interface{}, error) {
	return sendTCPCommand(conn, map[string]interface{}{"cmd": "run_prog"})
}

// stopProg stops the currently running program
func stopProg(conn net.Conn) (map[string]interface{}, error) {
	return sendTCPCommand(conn, map[string]interface{}{"cmd": "stop_prog"})
}

// setIndex sets the program index (line number)
// TODO: fix unnecessary type conversion / missmatch
func setIndex(conn net.Conn, index int) (map[string]interface{}, error) {
	return sendTCPCommand(conn, map[string]interface{}{
		"cmd":   "set_index",
		"index": index,
	})
}

// calibrate runs the calibrate command
func calibrate(conn net.Conn) (map[string]interface{}, error) {
	return sendTCPCommand(conn, map[string]interface{}{"cmd": "calibrate"})
}
