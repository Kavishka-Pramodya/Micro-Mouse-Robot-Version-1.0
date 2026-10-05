import urllib.request
import json
import time
from datetime import datetime

print("Starting robot data logger for 2 minutes...")
with open("robot_log.txt", "w") as f:
    for i in range(240): # 2 minutes (240 * 0.5s)
        try:
            req = urllib.request.Request("http://10.46.207.103/data")
            with urllib.request.urlopen(req, timeout=2) as response:
                data = json.loads(response.read().decode())
                log_line = f"{datetime.now().strftime('%H:%M:%S.%f')[:-3]} | F1:{data['f1_dist']} F2:{data['f2_dist']} L:{data['l_dist']} R:{data['r_dist']} | Yaw:{data['yaw']} | Stat:{data['s']}\n"
                f.write(log_line)
                f.flush()
        except Exception as e:
            f.write(f"{datetime.now().strftime('%H:%M:%S.%f')[:-3]} | Connection Error\n")
            f.flush()
        time.sleep(0.5)
print("Logging complete.")
