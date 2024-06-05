Codebase for SCI at UCLA Inertial Navigational Robot

Upload INROnBoard to BLE-compatible Arduino running simple differential-drive robot with MPU6050 attached

Run Python on computer for camera and BLE data collection with Bluepy

*note: btle.py changed

resp = self._waitResp(wantType + ['ntfy', 'ind']...
to
resp = self._waitResp(wantType + ['ntfy', 'ind', 'rd']...