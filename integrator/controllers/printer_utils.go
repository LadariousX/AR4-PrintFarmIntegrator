package controllers

import (
	"bytes"
	"encoding/json"
	"fmt"
	"integrator/core"
	"io"
	"log"
	"mime/multipart"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"time"
)

type OctoPrintClient struct {
	baseURL string
	apiKey  string
	client  *http.Client
}

func StartOctoPrintServ(appState *core.AppState, reset bool) error {
	if reset {
		appState.OctoStatus.Set("Octoprint status: restarting server")
		fmt.Println("restarting Octoprint server")
	}

	octoprintPath := os.Getenv("OCTOPRINT_PATH_FULL")
	out, _ := exec.Command("pgrep", "-f", "octoprint").Output()
	instances := strings.Split(string(out), "\n")
	if (len(instances)-1) != 1 || reset {
		if !reset {
			appState.OctoStatus.Set("Octoprint status: launching server")
			fmt.Println("starting Octoprint server")
		}
		if (len(instances)-1) > 1 || reset {
			for _, instance := range instances {
				_ = exec.Command("kill", "-9", instance).Run()
			}
		}
		cmd := exec.Command(octoprintPath, "serve")
		//cmd.Stdout = os.Stdout
		//cmd.Stderr = os.Stderr
		if err := cmd.Start(); err != nil {
			appState.OctoStatus.Set("Octoprint status: Error launching server, reattempting")
			return fmt.Errorf("error launching new Octoprint server instance, %w", err)
		}
		log.Println("started new OctoPrint instance")
		time.Sleep(15 * time.Second) // wait for octoprint to initialize before continuing.
	}
	return nil
}

// NewOctoPrint creates a new OctoPrint client
func NewOctoPrint() *OctoPrintClient {
	host, apiKey := os.Getenv("OCTOPRINT_HOSTNAME"), os.Getenv("OCTOPRINT_APP_API")
	host += ":5000"
	return &OctoPrintClient{
		baseURL: fmt.Sprintf("http://%s", host),
		apiKey:  apiKey,
		client: &http.Client{
			Timeout: 30 * time.Second,
		},
	}
}

// makeRequest is a helper for API calls
func (o *OctoPrintClient) makeRequest(method, endpoint string, body interface{}) ([]byte, error) {
	var reqBody io.Reader
	if body != nil {
		jsonData, err := json.Marshal(body)
		if err != nil {
			return nil, fmt.Errorf("octoprint: failed to marshal JSON: %v", err)
		}
		reqBody = bytes.NewBuffer(jsonData)
	}

	req, err := http.NewRequest(method, o.baseURL+endpoint, reqBody)
	if err != nil {
		return nil, fmt.Errorf("octoprint: failed to create request: %v", err)
	}

	req.Header.Set("X-Api-Key", o.apiKey)
	req.Header.Set("Content-Type", "application/json")

	resp, err := o.client.Do(req)
	if err != nil {
		return nil, fmt.Errorf("octoprint: request failed: %v", err)
	}
	defer func(Body io.ReadCloser) { _ = Body.Close() }(resp.Body)

	respBody, err := io.ReadAll(resp.Body)
	if err != nil {
		return nil, fmt.Errorf("octoprint: failed to read response: %v", err)
	}

	if resp.StatusCode >= 400 {
		return nil, fmt.Errorf("octoprint: API error %d: %s", resp.StatusCode, string(respBody))
	}

	return respBody, nil
}

// UploadFile uploads a G-code file to OctoPrint
func (o *OctoPrintClient) UploadFile(localPath string) error {
	log.Printf("octoprint: Uploading file %s", localPath)

	// Open file
	file, err := os.Open(localPath)
	if err != nil {
		return fmt.Errorf("octoprint: failed to open file: %v", err)
	}
	defer file.Close()

	// Create multipart form
	body := &bytes.Buffer{}
	writer := multipart.NewWriter(body)

	part, err := writer.CreateFormFile("file", filepath.Base(localPath))
	if err != nil {
		return fmt.Errorf("octoprint: failed to create form file: %v", err)
	}

	_, err = io.Copy(part, file)
	if err != nil {
		return fmt.Errorf("octoprint: failed to copy file data: %v", err)
	}

	if err := writer.WriteField("overwrite", "true"); err != nil {
		return fmt.Errorf("octoprint: failed to write overwrite field: %v", err)
	}
	writer.Close()

	// Upload
	req, err := http.NewRequest("POST", o.baseURL+"/api/files/local", body)
	if err != nil {
		return fmt.Errorf("octoprint: failed to create upload request: %v", err)
	}

	req.Header.Set("X-Api-Key", o.apiKey)
	req.Header.Set("Content-Type", writer.FormDataContentType())

	resp, err := o.client.Do(req)
	if err != nil {
		return fmt.Errorf("octoprint: upload request failed: %v", err)
	}
	defer resp.Body.Close()

	if resp.StatusCode != 201 {
		respBody, _ := io.ReadAll(resp.Body)
		return fmt.Errorf("octoprint: upload failed with status %d: %s", resp.StatusCode, string(respBody))
	}

	//log.Println("octoprint: Upload complete")
	return nil
}

// StartPrint starts printing a file
func (o *OctoPrintClient) StartPrint(filename string) error {
	payload := map[string]interface{}{
		"command": "select",
		"print":   true,
	}

	_, err := o.makeRequest("POST", "/api/files/local/"+filename, payload)
	if err != nil {
		return fmt.Errorf("octoprint: failed to start print: %v", err)
	}

	log.Println("octoprint: Print started")
	return nil
}

// PausePrint pauses the current print
func (o *OctoPrintClient) PausePrint() error {
	log.Println("octoprint: Pausing print")

	payload := map[string]string{
		"command": "pause",
		"action":  "pause",
	}

	_, err := o.makeRequest("POST", "/api/job", payload)
	if err != nil {
		return fmt.Errorf("octoprint: failed to pause: %v", err)
	}
	return nil
}

// ResumePrint resumes a paused print
func (o *OctoPrintClient) ResumePrint() error {
	log.Println("octoprint: Resuming print")

	payload := map[string]string{
		"command": "pause",
		"action":  "resume",
	}

	_, err := o.makeRequest("POST", "/api/job", payload)
	if err != nil {
		return fmt.Errorf("octoprint: failed to resume: %v", err)
	}
	return nil
}

// StopPrint cancels the current print
func (o *OctoPrintClient) StopPrint() error {
	log.Println("octoprint: Stopping print")

	payload := map[string]string{
		"command": "cancel",
	}

	_, err := o.makeRequest("POST", "/api/job", payload)
	if err != nil {
		return fmt.Errorf("octoprint: failed to stop: %v", err)
	}
	return nil
}

// GetJobResp gets printer status
func (o *OctoPrintClient) GetJobResp() (*core.JobResponse, error) {
	data, err := o.makeRequest("GET", "/api/job", nil)
	if err != nil {
		return nil, fmt.Errorf("octoprint: failed to get status: %v", err)
	}

	var status core.JobResponse
	if err := json.Unmarshal(data, &status); err != nil {
		return nil, fmt.Errorf("octoprint: failed to parse status: %v", err)
	}

	return &status, nil
}

// GetPrinterResp gets printer state and temperatures
func (o *OctoPrintClient) GetPrinterResp() (*core.PrinterResponse, error) {
	data, err := o.makeRequest("GET", "/api/printer", nil)
	if err != nil {
		return nil, fmt.Errorf("octoprint: failed to get printer state: %v", err)
	}

	var state core.PrinterResponse
	if err := json.Unmarshal(data, &state); err != nil {
		return nil, fmt.Errorf("octoprint: failed to parse printer state: %v", err)
	}

	return &state, nil
}

// SendGCode sends a single G-code command
// G28 home, M17 stepper Enable, M84 stepper disable.
func (o *OctoPrintClient) SendGCode(command string) error {
	payload := map[string]interface{}{
		"commands": []string{command},
	}

	_, err := o.makeRequest("POST", "/api/printer/command", payload)
	if err != nil {
		return fmt.Errorf("octoprint: failed to send G-code: %v", err)
	}
	return nil
}

// MoveToPosition moves printer to position
func (o *OctoPrintClient) MoveToPosition(x, y, z *float64, feedrate int) error {
	commands := []string{"G90"} // Absolute positioning

	cmd := "G1"
	if x != nil {
		cmd += fmt.Sprintf(" X%.2f", *x)
	}
	if y != nil {
		cmd += fmt.Sprintf(" Y%.2f", *y)
	}
	if z != nil {
		cmd += fmt.Sprintf(" Z%.2f", *z)
	}
	if feedrate > 0 {
		cmd += fmt.Sprintf(" F%d", feedrate)
	}

	commands = append(commands, cmd)

	payload := map[string]interface{}{
		"commands": commands,
	}

	log.Printf("octoprint: Moving to position: %s", cmd)
	_, err := o.makeRequest("POST", "/api/printer/command", payload)
	if err != nil {
		return fmt.Errorf("octoprint: failed to move: %v", err)
	}
	return nil
}
