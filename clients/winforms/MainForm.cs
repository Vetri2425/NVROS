using System;
using System.Text.Json;
using System.Windows.Forms;
using SocketIOClient;

namespace FlashRoverWinForms
{
    public class MainForm : Form
    {
        private readonly Label _lblConn = new() { AutoSize = true };
        private readonly Label _lblMode = new() { AutoSize = true };
        private readonly Label _lblStatus = new() { AutoSize = true };
        private readonly Label _lblBattery = new() { AutoSize = true };
        private readonly Label _lblGPS = new() { AutoSize = true };
        private readonly Label _lblLink = new() { AutoSize = true };
        private readonly Label _lblPos = new() { AutoSize = true };
        private readonly Label _lblAge = new() { AutoSize = true };

        private SocketIO? _socket;
        private string _backendUrl = Environment.GetEnvironmentVariable("JETSON_BACKEND_URL") ?? "http://192.168.1.100:5000";
        private DateTimeOffset? _lastUpdate;

        public MainForm()
        {
            Text = "Flash Rover - Telemetry";
            Width = 520;
            Height = 260;
            FormBorderStyle = FormBorderStyle.FixedSingle;
            MaximizeBox = false;

            var table = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 8,
                Padding = new Padding(10),
                AutoSize = true,
            };
            table.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 30));
            table.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 70));

            void AddRow(string name, Control valueCtrl, int row)
            {
                var lbl = new Label { Text = name, AutoSize = true };
                table.Controls.Add(lbl, 0, row);
                table.Controls.Add(valueCtrl, 1, row);
            }

            AddRow("Connection", _lblConn, 0);
            AddRow("Mode", _lblMode, 1);
            AddRow("Status", _lblStatus, 2);
            AddRow("Battery", _lblBattery, 3);
            AddRow("GPS", _lblGPS, 4);
            AddRow("Link", _lblLink, 5);
            AddRow("Position", _lblPos, 6);
            AddRow("Last Update", _lblAge, 7);

            Controls.Add(table);

            Shown += async (_, __) => await ConnectAsync();

            var ageTimer = new Timer { Interval = 1000, Enabled = true };
            ageTimer.Tick += (_, __) => UpdateAge();
        }

        private void UpdateAge()
        {
            if (_lastUpdate.HasValue)
            {
                var age = DateTimeOffset.Now - _lastUpdate.Value;
                _lblAge.Text = $"{(int)age.TotalSeconds}s ago";
            }
        }

        private async System.Threading.Tasks.Task ConnectAsync()
        {
            _lblConn.Text = $"Connecting to {_backendUrl} ...";
            _socket = new SocketIO(_backendUrl, new SocketIOOptions
            {
                Reconnection = true,
                ReconnectionAttempts = int.MaxValue,
                ReconnectionDelay = 2000,
                Transport = SocketIOClient.Transport.TransportProtocol.WebSocket
            });

            _socket.OnConnected += (_, __) => BeginInvoke(new Action(() => _lblConn.Text = "Backend: ONLINE"));
            _socket.OnDisconnected += (_, __) => BeginInvoke(new Action(() => _lblConn.Text = "Backend: OFFLINE"));

            _socket.On("connection_status", response =>
            {
                try
                {
                    var doc = JsonDocument.Parse(response.ToString());
                    var status = doc.RootElement.GetProperty("status").GetString();
                    BeginInvoke(new Action(() => _lblConn.Text = $"Vehicle: {status}"));
                }
                catch { }
            });

            _socket.On("rover_data", response =>
            {
                try
                {
                    var doc = JsonDocument.Parse(response.ToString());
                    var root = doc.RootElement;
                    var mode = root.TryGetProperty("mode", out var pm) ? pm.GetString() : "UNKNOWN";
                    var status = root.TryGetProperty("status", out var ps) ? ps.GetString() : "disarmed";
                    var battery = root.TryGetProperty("battery", out var pb) ? pb.GetInt32() : -1;
                    var rtk = root.TryGetProperty("rtk_status", out var pr) ? pr.GetString() : "N/A";
                    var hrms = root.TryGetProperty("hrms", out var ph) ? ph.GetString() : null;
                    var link = root.TryGetProperty("signal_strength", out var pl) ? pl.GetString() : "No Link";
                    var rcOk = root.TryGetProperty("rc_connected", out var prc) && prc.GetBoolean();

                    string pos = "N/A";
                    if (root.TryGetProperty("position", out var ppos) && ppos.ValueKind == JsonValueKind.Object)
                    {
                        var lat = ppos.TryGetProperty("lat", out var plat) ? plat.GetDouble() : double.NaN;
                        var lng = ppos.TryGetProperty("lng", out var plng) ? plng.GetDouble() : double.NaN;
                        pos = double.IsNaN(lat) ? "N/A" : $"{lat:F6}, {lng:F6}";
                    }

                    if (root.TryGetProperty("last_update", out var pu))
                    {
                        if (pu.ValueKind == JsonValueKind.Number)
                        {
                            _lastUpdate = DateTimeOffset.FromUnixTimeSeconds(pu.GetInt64());
                        }
                        else if (pu.ValueKind == JsonValueKind.String)
                        {
                            if (DateTimeOffset.TryParse(pu.GetString(), out var parsed))
                                _lastUpdate = parsed;
                        }
                    }

                    BeginInvoke(new Action(() =>
                    {
                        _lblMode.Text = mode ?? "UNKNOWN";
                        _lblStatus.Text = status ?? "disarmed";
                        _lblBattery.Text = battery >= 0 ? $"{battery}%" : "N/A";
                        _lblGPS.Text = hrms is not null ? $"{rtk} • {hrms}m" : rtk ?? "N/A";
                        _lblLink.Text = rcOk ? $"{link} • RC OK" : $"{link} • RC Lost";
                        _lblPos.Text = pos;
                        UpdateAge();
                    }));
                }
                catch { }
            });

            await _socket.ConnectAsync();
        }
    }
}

