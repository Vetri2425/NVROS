Flash Rover WinForms Client
===========================

Minimal Windows Forms client that connects to the Flask-Socket.IO backend and streams rover telemetry in real time.

Prerequisites
- Windows 10/11
- .NET 6 SDK or newer
- NuGet access to restore packages

Configure
- Backend URL via environment variable (recommended):
  - `JETSON_BACKEND_URL=http://<backend-host>:5000`
  - Defaults to `http://192.168.1.100:5000` if not set.

Build & Run
1. Open a Developer PowerShell/Command Prompt in `clients/winforms`
2. Restore and build:
   - `dotnet restore`
   - `dotnet build`
3. Run:
   - `dotnet run`

Notes
- Uses the `SocketIOClient` NuGet package to connect to the existing Socket.IO server.
- Displays connection status, mode/armed, battery, GPS/RTK, link strength, position, and last update age.
- UI updates are marshaled onto the UI thread via `BeginInvoke`.

