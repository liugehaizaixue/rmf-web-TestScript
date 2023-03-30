import yaml

def GetWaypoints():
    "test 获取带名字的waypoint"
    file_path = './Setting/airport_terminal.building.yaml'
    with open(file_path, 'r') as f:
        yaml_content = f.read()
    # 解析YAML内容
    yaml_data = yaml.safe_load(yaml_content)
    points = yaml_data["levels"]["L1"]["vertices"]
    waypoints=[]
    for inner_arr in points:
        if inner_arr[3] != '':
            waypoints.append(inner_arr[3])
    #  获取地图所有waypoints
    waypoints=waypoints
    # waypoints.remove("lift1")
    print(waypoints)
  
GetWaypoints()