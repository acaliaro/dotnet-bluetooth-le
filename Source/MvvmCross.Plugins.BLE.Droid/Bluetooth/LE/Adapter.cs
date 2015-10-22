using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Android.App;
using Android.Bluetooth;
using Android.Content;
using MvvmCross.Plugins.BLE.Bluetooth.LE;

namespace MvvmCross.Plugins.BLE.Droid.Bluetooth.LE
{
    /// <summary>
    /// TODO: this really should be a singleton.
    /// </summary>
    public class Adapter : Java.Lang.Object, BluetoothAdapter.ILeScanCallback, IAdapter
    {
        // events
        public event EventHandler<DeviceDiscoveredEventArgs> DeviceDiscovered = delegate { };
        public event EventHandler<DeviceConnectionEventArgs> DeviceConnected = delegate { };
        public event EventHandler<DeviceBondStateChangedEventArgs> DeviceBondStateChanged = delegate { };
        public event EventHandler<DeviceConnectionEventArgs> DeviceDisconnected = delegate { };
        public event EventHandler ScanTimeoutElapsed = delegate { };

        // class members
        protected BluetoothManager _manager;
        protected BluetoothAdapter _adapter;
        protected GattCallback _gattCallback;

        public bool IsScanning
        {
            get { return this._isScanning; }
        }

        public int ScanTimeout { get; set; }
        protected bool _isScanning;

        public IList<IDevice> DiscoveredDevices
        {
            get
            {
                return this._discoveredDevices;
            }
        } protected IList<IDevice> _discoveredDevices = new List<IDevice>();

        public IList<IDevice> ConnectedDevices
        {
            get
            {
                return this._connectedDevices;
            }
        } protected IList<IDevice> _connectedDevices = new List<IDevice>();

        public Dictionary<string, IDevice> DeviceRegistry { get; private set; }

        public Adapter()
        {
            var appContext = Android.App.Application.Context;
            // get a reference to the bluetooth system service
            this._manager = (BluetoothManager)appContext.GetSystemService("bluetooth");
            this._adapter = this._manager.Adapter;

            this._gattCallback = new GattCallback(this);

            this._gattCallback.DeviceConnected += (object sender, DeviceConnectionEventArgs e) =>
            {
                this._connectedDevices.Add(e.Device);
                this.DeviceConnected(this, e);
            };

            this._gattCallback.DeviceDisconnected += (object sender, DeviceConnectionEventArgs e) =>
            {
                RemoveDeviceFromList(e.Device);
                this.DeviceDisconnected(this, e);
            };

            var bondStatusBroadcastReceiver = new BondStatusBroadcastReceiver();
            Application.Context.RegisterReceiver(bondStatusBroadcastReceiver,
                new IntentFilter(BluetoothDevice.ActionBondStateChanged));

            bondStatusBroadcastReceiver.BondStateChanged += (s, args) =>
            {
                this.DeviceBondStateChanged(this, args);
            };

            ScanTimeout = 10000;
            DeviceRegistry = new Dictionary<string, IDevice>();
        }

        //TODO: scan for specific service type eg. HeartRateMonitor
        public async void StartScanningForDevices(Guid serviceUuid)
        {
            StartScanningForDevices();
            //			throw new NotImplementedException ("Not implemented on Android yet, look at _adapter.StartLeScan() overload");
        }
        public async void StartScanningForDevices()
        {
            if (_isScanning)
            {
                Console.WriteLine("Adapter: Already scanning.");
                return;
            }

            Console.WriteLine("Adapter: Starting a scan for devices.");

            // clear out the list
            this._discoveredDevices = new List<IDevice>();

            // start scanning
            this._isScanning = true;
            this._adapter.StartLeScan(this);

            // in 10 seconds, stop the scan
            await Task.Delay(ScanTimeout);

            // if we're still scanning
            if (this._isScanning)
            {
                Console.WriteLine("Adapter: Scan timeout has elapsed.");
                StopScanningForDevices();
                this.ScanTimeoutElapsed(this, new EventArgs());
            }
        }

        public void StopScanningForDevices()
        {
            if (this._isScanning)
            {
                Console.WriteLine("Adapter: Stopping the scan for devices.");
                this._isScanning = false;
                this._adapter.StopLeScan(this);
            }
            else
            {
                Console.WriteLine("Adapter: Allready stopped scan.");
            }
        }

        public void OnLeScan(BluetoothDevice bleDevice, int rssi, byte[] scanRecord)
        {
            Console.WriteLine("Adapter.LeScanCallback: " + bleDevice.Name);
            // TODO: for some reason, this doesn't work, even though they have the same pointer,
            // it thinks that the item doesn't exist. so i had to write my own implementation
            //			if(!this._discoveredDevices.Contains(device) ) {
            //				this._discoveredDevices.Add (device );
            //			}
            Device device = new Device(bleDevice, null, null, rssi);

            if (!DeviceExistsInDiscoveredList(bleDevice))
                this._discoveredDevices.Add(device);
            // TODO: in the cross platform API, cache the RSSI
            // TODO: shouldn't i only raise this if it's not already in the list?
            this.DeviceDiscovered(this, new DeviceDiscoveredEventArgs { Device = device });
        }

        protected bool DeviceExistsInDiscoveredList(BluetoothDevice device)
        {
            return this._discoveredDevices.Any(d => device.Address == ((BluetoothDevice)d.NativeDevice).Address);
        }


        public void ConnectToDevice(IDevice device)
        {
            // returns the BluetoothGatt, which is the API for BLE stuff
            // TERRIBLE API design on the part of google here.
            AddToDeviceRegistry(device);

            ((BluetoothDevice)device.NativeDevice).ConnectGatt(Android.App.Application.Context, true, this._gattCallback);
        }

        private void AddToDeviceRegistry(IDevice device)
        {
            var nativeDevice = ((BluetoothDevice)device.NativeDevice);
            if (!DeviceRegistry.ContainsKey(nativeDevice.Address))
            {
                DeviceRegistry.Add(nativeDevice.Address, device);
            }
        }

        public void CreateBondToDevice(IDevice device)
        {
            ((BluetoothDevice)device.NativeDevice).CreateBond();
        }

        public void DisconnectDevice(IDevice deviceToDisconnect)
        {
            //make sure everything is disconnected
            AddToDeviceRegistry(deviceToDisconnect);
            ((Device)deviceToDisconnect).Disconnect();
        }

        /// <summary>
        /// Removes a device with the given id from the list
        /// </summary>
        /// <param name="deviceToDisconnect"></param>
        private void RemoveDeviceFromList(IDevice deviceToDisconnect)
        {
            var device = this._connectedDevices.FirstOrDefault(d => d.ID.Equals(deviceToDisconnect.ID));
            if (device != null)
            {
                this._connectedDevices.Remove(device);
            }
        }
    }


    [BroadcastReceiver()]
    public class BondStatusBroadcastReceiver : BroadcastReceiver
    {
        public event EventHandler<DeviceBondStateChangedEventArgs> BondStateChanged;

        public override void OnReceive(Context context, Intent intent)
        {
            var bondState = (Bond)intent.GetIntExtra(BluetoothDevice.ExtraBondState, (int)Bond.None);
            var device = new Device((BluetoothDevice)intent.GetParcelableExtra(BluetoothDevice.ExtraDevice), null, null, 0);
            Console.WriteLine(bondState.ToString());

            if (BondStateChanged == null) return;

            switch (bondState)
            {
                case Bond.None:
                    BondStateChanged(this, new DeviceBondStateChangedEventArgs() { Device = device, State = DeviceBondState.NotBonded });
                    break;

                case Bond.Bonding:
                    BondStateChanged(this, new DeviceBondStateChangedEventArgs() { Device = device, State = DeviceBondState.Bonding });
                    break;

                case Bond.Bonded:
                    BondStateChanged(this, new DeviceBondStateChangedEventArgs() { Device = device, State = DeviceBondState.Bonded });
                    break;

            }
        }
    }
}
