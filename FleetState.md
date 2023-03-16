FleetState 返回的json数据格式如下：

```json
{
	'name': 'tinyRobot',
	'robots': {
		'tinyRobot1': {
			'name': 'tinyRobot1',
			'status': 'working',
			'task_id': 'pickup.dispatch-0',
			'unix_millis_time': 1678872552260,
			'location': {
				'map': 'L1',
				'x': 5.650000095367432,
				'y': -13.15250015258789,
				'yaw': 1.5707963705062866
			},
			'battery': 1.0,
			'issues': []
		}
	}
}
```