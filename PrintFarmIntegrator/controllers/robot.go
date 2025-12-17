package controllers

import (
	"PrintFarmIntegrator/core"

	"encoding/json"
	"fmt"
	"log"
	"net"
	"os"
	"os/exec"
	"strings"
	"time"
)

func AR4Controller(state *core.AppState, control *core.RCtrl) {
	var conn net.Conn
	var err error
	var reset = false
	var robotProg = ""
	print(robotProg)
	for { // main robot loop & go routine
		if conn == nil {
			state.ArmStatus.Set("Arm status: connecting")
			conn, err = initConnectAR4(reset)
			reset = false

			if err != nil {
				log.Println("AR4 connection failed: ", err)
				state.ArmStatus.Set("Arm status: disconnected")
				continue
			}

			log.Println("AR4 connected")
			state.ArmStatus.Set("Arm status: connected")
		}

		select {
		case <-control.AR4CtrlReset:
			log.Println("AR4 reset requested")
			state.ArmStatus.Set("Arm status: resetting")
			conn = closeConn(conn)
			reset = true
			// Do NOT reconnect here — loop will handle it

		case <-control.Start:
			log.Println("Start job requested")
			// send start command over TCP (later)

		case <-control.Pause:
			log.Println("Pause requested")
			// send pause command (later)

		case <-control.Stop:
			log.Println("Stop requested")
			// send stop command (later)

		case path := <-control.SetRobotProg:
			robotProg = path

		default: // TCP HEALTH / DATA
			// Non-blocking TCP check placeholder
			_ = conn.SetReadDeadline(time.Now().Add(100 * time.Millisecond))
			buf := make([]byte, 1)
			if _, err := conn.Read(buf); err != nil {
				// Only close on real errors, not timeout
				if netErr, ok := err.(net.Error); !ok || !netErr.Timeout() {
					log.Println("AR4Ctrl TCP lost:", err)
					state.ArmStatus.Set("Arm status: disconnected")
					conn = closeConn(conn)
				}
			}
		}
	}
}

func initConnectAR4(reset bool) (net.Conn, error) {
	if reset {
		fmt.Println("restarting AR4Ctrl software")
	}
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
		launchErr := launchAR4Control(pythonPath, scriptPath)
		if launchErr != nil {
			fmt.Println(launchErr)
			return nil, launchErr
		}
	}

	// connect tcp
	return connectTCP()

}

func launchAR4Control(pythonPath, scriptPath string) error {
	cmd := exec.Command(pythonPath, scriptPath)
	cmd.Stdout = os.Stdout
	cmd.Stderr = os.Stderr
	if err := cmd.Start(); err != nil {
		return fmt.Errorf("error launching new AR4Control instance, %w", err)
	}
	log.Println("started new AR4Control instance")
	return nil
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

	// Send command with newline delimiter
	cmdBytes = append(cmdBytes, '\n')
	if _, err := conn.Write(cmdBytes); err != nil {
		return nil, fmt.Errorf("failed to send command: %w", err)
	}

	// Read response with timeout
	conn.SetReadDeadline(time.Now().Add(2 * time.Second))
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

// runFile loads and runs a program file
func runFile(conn net.Conn, filePath string) (map[string]interface{}, error) {
	return sendTCPCommand(conn, map[string]interface{}{
		"cmd":  "run_file",
		"path": filePath,
	})
}

// stopProg stops the currently running program
func stopProg(conn net.Conn) (map[string]interface{}, error) {
	return sendTCPCommand(conn, map[string]interface{}{"cmd": "stop_prog"})
}

// resProg resumes the program
func resProg(conn net.Conn) (map[string]interface{}, error) {
	return sendTCPCommand(conn, map[string]interface{}{"cmd": "res_prog"})
}
