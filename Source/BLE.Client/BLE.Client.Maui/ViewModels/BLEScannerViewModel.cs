using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Diagnostics;
using System.Diagnostics.Metrics;
using System.Runtime.CompilerServices;
using System.Threading;
using System.Windows.Input;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using Plugin.BLE;
using Plugin.BLE.Abstractions.Contracts;
using Plugin.BLE.Abstractions.EventArgs;
using Plugin.BLE.Abstractions.Extensions;

namespace BLE.Client.Maui.ViewModels
{
    public partial class BLEScannerViewModel : ObservableObject
    {
        private const string ConfigurationBleService = "04000000-e814-4b55-bbca-2c58013d29f8";
        private const string ConfigurationCharWrite = "04010000-e814-4b55-bbca-2c58013d29f8";
        private const string ConfigurationCharIndication = "04020000-e814-4b55-bbca-2c58013d29f8";
        private CancellationTokenSource _cancellationToken;
        private ICharacteristic _configurationIndicationChar;
        private ICharacteristic _configurationWriteChar;
        protected IService ConfigurationService;
        [ObservableProperty]
        bool _permissionsGranted;

        IDispatcherTimer timer = Application.Current.Dispatcher.CreateTimer();

        [ObservableProperty]
        [NotifyPropertyChangedFor(nameof(IsStateOn))]
        private IBluetoothLE _bluetoothManager;

        private IAdapter Adapter;

        [ObservableProperty]
        bool _isStateOn;
        [ObservableProperty]
        string _stateText;
        [ObservableProperty]
        bool _isRefreshing;
        [ObservableProperty]
        bool _isConnected;

        CancellationTokenSource _scanCancellationTokenSource = new();
        readonly CancellationToken _scanCancellationToken;

        [ObservableProperty]
        private bool _isScanning = false;

        [ObservableProperty]
        BLEDeviceViewModel _selectedItem;

        [ObservableProperty]
        public ObservableCollection<BLEDeviceViewModel> _bLEDevices = [];
        [ObservableProperty]
        private ObservableCollection<string> _messages = [];
        [ObservableProperty]
        string _scanState;
        [ObservableProperty]
        string _scanLabelText = "Start Scan";


        public BLEScannerViewModel()
        {
            DebugMessage($"Into BLEScannerViewModel constructor");
            _scanCancellationToken = _scanCancellationTokenSource.Token;
            ConfigureBLE();

            timer.Interval = TimeSpan.FromSeconds(5);
            timer.Tick += (s, e) =>
            {
                if (!_canWrite)
                    return;

                byte[] buffer = GetEVSEVerboseStateRequest();

                MainThread.BeginInvokeOnMainThread(async () =>
                {
                    try
                    {
                        DebugMessage("---> SEND " + BitConverter.ToString(buffer) + " counter = " + countTimer);
                        countTimer++;

                        await _configurationWriteChar.WriteAsync(buffer);
                        _canWrite = false;
                    }
                    catch (Exception ex)
                    {
                        DebugMessage(ex.Message);
                    }
                });

            };

            MainThread.InvokeOnMainThreadAsync(async () =>
            {
                PermissionsGranted = await CheckCorrectPermissionsAsync();
            });
        }

        [RelayCommand]
        async Task GrantPermissionsAsync()
        {
            try
            {
                if (await CheckCorrectPermissionsAsync() == false)
                    await AskCorrectPermissionsAsync();
            }
            catch (Exception ex)
            {
                DebugMessage(ex.Message);
            }
        }


        partial void OnIsScanningChanged(bool value)
        {
            ScanState = value ? "Scanning" : "Waiting";
            DebugMessage($"Getting ScanState: '{ScanState}'");

            ScanLabelText = value ? "Cancel" : "Start Scan";
            DebugMessage($"Getting ScanLabelText: '{ScanLabelText}'");
        }

        private void ClearMessages()
        {
            DebugMessage($"enter ClearMessages");
            Messages.Clear();
            DebugMessage($"exit ClearMessages");
        }
        private string GetStateText()
        {
            DebugMessage("Into GetState");
            var result = "Unknown BLE state.";

            IsStateOn = BluetoothManager.IsOn;

            switch (BluetoothManager.State)
            {
                case BluetoothState.Unknown:
                    result = "Unknown BLE state.";
                    break;
                case BluetoothState.Unavailable:
                    result = "BLE is not available on this device.";
                    break;
                case BluetoothState.Unauthorized:
                    result = "You are not allowed to use BLE.";
                    break;
                case BluetoothState.TurningOn:
                    result = "BLE is warming up, please wait.";
                    break;
                case BluetoothState.On:
                    result = "BLE is on.";
                    break;
                case BluetoothState.TurningOff:
                    result = "BLE is turning off. That's sad!";
                    break;
                case BluetoothState.Off:
                    result = "BLE is off. Turn it on!";
                    break;
            }
            DebugMessage($"return state as '{result}'");

            return result;
        }

        private void ShowMessage(string message)
        {
            DebugMessage(message);
            App.AlertSvc.ShowAlert("BLE Scanner", message);
        }

        private void DebugMessage(string message)
        {
            Debug.WriteLine(message);
            Messages.Insert(0, message);
            OnMessageAdded?.Invoke();
        }

        public Action OnMessageAdded;

        private void ConfigureBLE()
        {
            DebugMessage("into ConfigureBLE");
            BluetoothManager = CrossBluetoothLE.Current;
            DebugMessage("got _bluetoothManager");
            if (BluetoothManager == null)
            {
                DebugMessage("CrossBluetoothLE.Current is null");
            }
            else
            {
                BluetoothManager.StateChanged += OnStateChanged;
            }

            Adapter = CrossBluetoothLE.Current.Adapter;
            if (Adapter == null)
            {
                DebugMessage("CrossBluetoothLE.Current.Adapter is null");
            }
            else
            {
                DebugMessage("go and set event handlers");

                AddAdapterEvents();

                Adapter.ScanMode = ScanMode.LowLatency;

                DebugMessage("event handlers set");
            }

            if (BluetoothManager == null && Adapter == null)
            {
                ShowMessage("Bluetooth and Adapter are both null");
            }
            else if (BluetoothManager == null)
            {
                ShowMessage("Bluetooth is null");
            }
            else if (Adapter == null)
            {
                ShowMessage("Adapter is null");
            }
        }

        private async void OnDeviceConnectionError(object sender, DeviceErrorEventArgs e)
        {
            DebugMessage("OnDeviceConnectionError");
            await StopBleCommunicationAsync();
        }

        private async void OnDeviceConnectionLost(object sender, DeviceErrorEventArgs e)
        {
            DebugMessage("OnDeviceConnectionLost");
            await StopBleCommunicationAsync();
        }

        private async void OnDeviceDisconnected(object sender, DeviceEventArgs e)
        {
            DebugMessage("OnDeviceDisconnected");
            await StopBleCommunicationAsync();
        }

        private byte[] GetMessageBytes(ushort command, byte[] payloadBytes)
        {
            var head = HeadBytes();
            var commandBytes = BitConverter.GetBytes(command);
            var len = BitConverter.GetBytes((ushort)payloadBytes.Length);
            var dataArray = head.Concat(commandBytes).Concat(len).Concat(payloadBytes);
            var crc = Calc_CRC_8(dataArray.ToArray(), dataArray.Count());
            return dataArray.Append(crc).ToArray();
        }

        private static byte[] HeadBytes()
        {
            return new byte[] { 0xda, 0x2e, };
        }

        public static byte Calc_CRC_8(byte[] addr, int len)
        {
            byte crc = 0;
            for (int i = 0; i < len; i++)
            {
                byte inbyte = addr[i];
                for (int j = 0; j < 8; j++)
                {
                    byte mix = (byte)((byte)(crc ^ inbyte) & 0x01);
                    crc >>= 1;
                    if (mix != 0)
                        crc ^= 0x8C;
                    inbyte >>= 1;
                }
            }

            return crc;
        }

        public byte[] GetEVSEVerboseStateRequest()
        {
            return MessageWithEmptyPayload(0x1220);
        }

        private byte[] MessageWithEmptyPayload(ushort command)
        {
            return GetMessageBytes(command, Array.Empty<byte>());
        }

        int countTimer = 1;
        private async void OnDeviceConnected(object sender, DeviceEventArgs e)
        {
            DebugMessage("OnDeviceConnected");

            await StartBleCommunicationAsync(e);
        }

        bool _canWrite = false;

        private async Task StartBleCommunicationAsync(DeviceEventArgs e)
        {
            IsConnected = true;

            ConfigurationService = await e.Device.GetServiceAsync(Guid.Parse(ConfigurationBleService));
            _configurationWriteChar = await ConfigurationService.GetCharacteristicAsync(Guid.Parse(ConfigurationCharWrite));
            _configurationIndicationChar = await ConfigurationService.GetCharacteristicAsync(Guid.Parse(ConfigurationCharIndication));

            _configurationIndicationChar.ValueUpdated -= OnConfigurationIndicationCharOnValueUpdated;

            _configurationIndicationChar.ValueUpdated += OnConfigurationIndicationCharOnValueUpdated;
            await _configurationIndicationChar.StartUpdatesAsync();

            await Task.Delay(1000);
            countTimer = 1;
            timer.Start();
            _canWrite = true;


            BLEDevices.Clear();
        }

        private async Task StopBleCommunicationAsync()
        {
            timer.Stop();
            IsConnected = false;

            SelectedItem = null;

            if (_configurationIndicationChar != null)
            {
                try
                {
                    await _configurationIndicationChar.StopUpdatesAsync();
                    _configurationIndicationChar.ValueUpdated -= OnConfigurationIndicationCharOnValueUpdated;
                }
                catch (Exception ex)
                {

                    DebugMessage("StopBleCommunicationAsync exception = " + ex.Message);
                }
            }

        }

        private void OnConfigurationIndicationCharOnValueUpdated(object sender, CharacteristicUpdatedEventArgs e)
        {
            if (e != null && e.Characteristic.Value.Length != 0)
            {
                DebugMessage("<--- RECV " + BitConverter.ToString(e.Characteristic.Value));
                _canWrite = true;
            }
        }

        private void OnStateChanged(object sender, BluetoothStateChangedArgs e)
        {

            DebugMessage("OnStateChanged");

            StateText = GetStateText();

            DebugMessage("OnStateChanged done");
        }

        private void Adapter_ScanTimeoutElapsed(object sender, EventArgs e)
        {
            DebugMessage("Adapter_ScanTimeoutElapsed");
            IsRefreshing = Adapter?.IsScanning ?? false;
            CleanupCancellationToken();
            DebugMessage("Adapter_ScanTimeoutElapsed done");
        }

        private void OnDeviceDiscovered(object sender, DeviceEventArgs args)
        {
            DebugMessage("OnDeviceDiscovered");
            AddOrUpdateDevice(args.Device);

            if (args.Device.NativeDevice != null && args.Device.Name != null)
            {
                DebugMessage("OnDeviceDiscovered args.Device.Name = " + args.Device.Name);

                var device = args.Device;
                var nativeDevice = args.Device.NativeDevice;

                var advertisementRecords = device.AdvertisementRecords;

                DebugMessage("OnDeviceDiscovered device.GetType() = " + device.GetType());
                DebugMessage("OnDeviceDiscovered nativeDevice.GetType() = " + nativeDevice.GetType());
                if (advertisementRecords != null)
                {
                    DebugMessage("OnDeviceDiscovered advertisementRecords.GetType() = " + advertisementRecords.GetType());
                    DebugMessage("OnDeviceDiscovered advertisementRecords.Count = " + advertisementRecords.Count);

                    foreach (var r in advertisementRecords)
                    {
                        if (r.Type == Plugin.BLE.Abstractions.AdvertisementRecordType.ShortLocalName)
                        {
                            string bitString = BitConverter.ToString(r.Data);

                            DebugMessage("OnDeviceDiscovered advertisementRecords.ShortLocalName = " + bitString);

                        }
                        else if (r.Type == Plugin.BLE.Abstractions.AdvertisementRecordType.CompleteLocalName)
                        {
                            string bitString = BitConverter.ToString(r.Data);

                            DebugMessage("OnDeviceDiscovered advertisementRecords.CompleteLocalName = " + bitString);
                        }
                    }
                }

            }

            DebugMessage("OnDeviceDiscovered done");
        }

        private void AddOrUpdateDevice(IDevice device)
        {
            DebugMessage($"Device Found: '{device.Id}'");
            var vm = BLEDevices.FirstOrDefault(d => d.DeviceId == device.Id);
            if (vm != null)
            {
                DebugMessage($"Update Device: {device.Id}");
            }
            else
            {
                if (device != null && device.Name != null && device.Name.ToUpper().StartsWith("DAZE"))
                {
                    DebugMessage($"Add Device: {device.Id}");
                    vm = new BLEDeviceViewModel(device);
                    MainThread.BeginInvokeOnMainThread(() => BLEDevices.Add(vm));
                }
            }
            DebugMessage($"Device Found: '{device.Id}' done");
        }

        private void CleanupCancellationToken()
        {
            DebugMessage("CleanUpCancellationToken");
            _scanCancellationTokenSource.Dispose();
            _scanCancellationTokenSource = null;
            IsScanning = false;
            DebugMessage("CleanUpCancellationToken done");
        }

        [RelayCommand]
        async Task DisconnectAsync()
        {
            if (SelectedItem != null)
            {
                await Adapter.DisconnectDeviceAsync(SelectedItem.CurrentDevice);
            }
        }

        [RelayCommand]
        async Task SelectDeviceAsync()
        {

            if (SelectedItem != null)
            {
                await Adapter.ConnectToDeviceAsync(SelectedItem.CurrentDevice);
            }
        }

        [RelayCommand]
        async Task ScanForDevicesAsync()
        {
            ClearMessages();
            if (!IsScanning)
            {
                IsScanning = true;
                DebugMessage($"Starting Scanning");
                StartScanForDevices();
                DebugMessage($"Started Scan");
            }
            else
            {
                DebugMessage($"Stopping Scan");
                _scanCancellationTokenSource.Cancel();
                IsScanning = false;
                DebugMessage($"Stop Scanning");
            }
        }
        private async Task UpdateConnectedDevices()
        {
            foreach (var connectedDevice in Adapter.ConnectedDevices)
            {
                //update rssi for already connected devices (so tha 0 is not shown in the list)
                try
                {
                    await connectedDevice.UpdateRssiAsync();
                }
                catch (Exception ex)
                {
                    ShowMessage($"Failed to update RSSI for {connectedDevice.Name}. Error: {ex.Message}");
                }

                AddOrUpdateDevice(connectedDevice);
            }
        }

        private async void StartScanForDevices()
        {
            DebugMessage("into StartScanForDevices");
            if (!await CheckCorrectPermissionsAsync())
            {
                if (!await AskCorrectPermissionsAsync())
                {
                    DebugMessage("Permissons fail - can't scan");
                    return;
                }
            }

            DebugMessage("StartScanForDevices called");
            BLEDevices.Clear();
            await UpdateConnectedDevices();
            OnPropertyChanged(nameof(BLEDevices));

            _scanCancellationTokenSource = new CancellationTokenSource();
            Adapter.ScanMode = ScanMode.LowLatency;


            RemoveAdapterEvents();
            AddAdapterEvents();

            Adapter.ScanMode = ScanMode.LowLatency;

            DebugMessage("call Adapter.StartScanningForDevicesAsync");
            await Adapter.StartScanningForDevicesAsync(_scanCancellationTokenSource.Token);
            DebugMessage("back from Adapter.StartScanningForDevicesAsync");
        }

        private void AddAdapterEvents()
        {
            Adapter.DeviceDiscovered += OnDeviceDiscovered;
            Adapter.DeviceAdvertised += OnDeviceDiscovered;
            Adapter.ScanTimeoutElapsed += Adapter_ScanTimeoutElapsed;
            Adapter.DeviceConnected += OnDeviceConnected;
            Adapter.DeviceDisconnected += OnDeviceDisconnected;
            Adapter.DeviceConnectionLost += OnDeviceConnectionLost;
            Adapter.DeviceConnectionError += OnDeviceConnectionError;
        }

        private void RemoveAdapterEvents()
        {
            Adapter.DeviceDiscovered -= OnDeviceDiscovered;
            Adapter.DeviceAdvertised -= OnDeviceDiscovered;
            Adapter.ScanTimeoutElapsed -= Adapter_ScanTimeoutElapsed;
            Adapter.DeviceConnected -= OnDeviceConnected;
            Adapter.DeviceDisconnected -= OnDeviceDisconnected;
            Adapter.DeviceConnectionLost -= OnDeviceConnectionLost;
            Adapter.DeviceConnectionError -= OnDeviceConnectionError;
        }



        private async Task<bool> AskCorrectPermissionsAsync()
        {
            DebugMessage("Into AskCorrectPermissionsAsync");
            var permissionResult = await Permissions.RequestAsync<Permissions.Bluetooth>();
            DebugMessage($"Back from await App.PlatformHelper: '{permissionResult}'");
            if (permissionResult != PermissionStatus.Granted)
            {
                DebugMessage($"!!Permissions denied!! '{permissionResult}'");
                ShowMessage("Permission denied. Not scanning.");
                AppInfo.ShowSettingsUI();
                return false;
            }

            DebugMessage("Exit AskCorrectPermissionsAsync");
            return true;
        }

        private async Task<bool> CheckCorrectPermissionsAsync()
        {
            DebugMessage("Into CheckCorrectPermissionsAsync");
            var permissionResult = await Permissions.CheckStatusAsync<Permissions.Bluetooth>();

            DebugMessage("Exit CheckCorrectPermissionsAsync");
            return permissionResult != PermissionStatus.Granted;
        }

    }
}
