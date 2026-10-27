from pathlib import Path

p = Path('/wa1/bridge_ws/src/ros1_bridge/include/ros1_bridge/factory.hpp')
source = p.read_text()
old = '    return node.subscribe(ops);'
assert source.count(old) == 1, 'Upstream subscription API changed'
source = source.replace(old, '    ops.transport_hints = ros::TransportHints().tcpNoDelay();\n' + old)
p.write_text(source)

# Upstream 611755f only recognizes [] in services; Pose[2] becomes the invalid
# C++ type geometry_msgs::Pose[2. Preserve fixed arrays and convert each element.
p = Path('/wa1/bridge_ws/src/ros1_bridge/ros1_bridge/__init__.py')
source = p.read_text()
old = "'array': True if '[]' in ros1_type else False,"
assert source.count(old) == 1
source = source.replace(old, "'array': '[' in ros1_type,\n                    'dynamic_array': '[]' in ros1_type,")
for side in ('ros1', 'ros2'):
    old = side + "_type.rstrip('[]')"
    assert source.count(old) == 2
    source = source.replace(old, side + "_type.split('[', 1)[0]")
p.write_text(source)

p = Path('/wa1/bridge_ws/src/ros1_bridge/resource/interface_factories.cpp.em')
source = p.read_text()
old = '  req@(to).@(field["ros" + to]["name"]).resize(req@(frm).@(field["ros" + frm]["name"]).size());'
assert source.count(old) == 1
source = source.replace(old, '@[          if field["dynamic_array"]]@\n' + old + '\n@[          end if]@')
p.write_text(source)
