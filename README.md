# Testing

- `workspace_ccstheia/` - CCS Theia workspace for the IWR6843 radar firmware (3D people tracking with fall, entry and exit detection). Open this folder as the CCS workspace.
- `Silvershield-mmWave/` - nRF5340 app (Zephyr / nRF Connect SDK 3.4.0) that reads the radar's UART messages, runs acoustic impact and "help" detection with Edge Impulse, and sends events over BLE.
