import requests
import json
import time

IP_WEBCAM_ADDRESS = "192.168.0.109:8080"
url = "http://" + IP_WEBCAM_ADDRESS + "/sensors.json"

while True:
    response = requests.get(url, verify=False)
    # print(response.content)
    # response.json()['rot_vector']['data'][0][1][0]
    # , json.dumps(response.json()['rot_vector']['data'][-1][1][1]*10), j
    vals = round(response.json()['gyro']['data'][-1][1][0]*-10, 5), round(response.json()['gyro']['data'][-1][1][1]*-10, 5)
    print(vals[0], vals[1])
    time.sleep(0.2)
    # print(time.time()-start)
# rotation = Vector3(json.dumps(response.json()['rot_vector']['data'][0][1][0]), json.dumps(response.json()['rot_vector']['data'][0][1][1]), json.dumps(response.json()['rot_vector']['data'][0][1][2]))