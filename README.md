<div align="center">
<img width="1200" height="475" alt="GHBanner" src="https://github.com/user-attachments/assets/0aa67016-6eaf-458a-adb2-6e31a0763ed6" />
</div>

# Run and deploy your AI Studio app

This contains everything you need to run your app locally.

View your app in AI Studio: https://ai.studio/apps/drive/1VDDKRXkmFj6FGzegMAZaaGr5zkdZUD6c

## Run Locally

**Prerequisites:**  Node.js, Python 3.10+, access to a MAVLink-enabled rover (update the connection string in `backend-python/server.py` as needed)

### 1. Install frontend dependencies

```bash
npm install
```

### 2. Configure environment variables

- Set the `GEMINI_API_KEY` in `.env.local` to your Gemini API key (if required by the UI).
- Ensure `.env` points `VITE_JETSON_BACKEND_URL` to your Python backend (default `http://localhost:5000`).

### 3. Start the real rover backend (Python)

```bash
cd backend-python
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python server.py
```

The backend connects to the rover via MAVLink and streams genuine telemetry to the UI.

### 4. Run the frontend

```bash
npm run dev
```

Open the provided Vite dev server URL in your browser to use the planner.

## Deploy

Deploy the frontend as you would a standard Vite React app. Host the Python backend separately close to your rover control network, keeping the Socket.IO endpoint accessible to the UI.
# NRP_ROS
# NRP_ROS
