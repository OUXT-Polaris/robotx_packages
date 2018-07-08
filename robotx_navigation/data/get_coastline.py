import overpy
import yaml

def fetch_result(tag,lat,lon):
    api = overpy.Overpass()
    coastline_request = "way(" + str(lat-0.005) + "," + str(lon-0.005) + "," + str(lat+0.005) + "," + str(lon+0.005) + "[coastline]);out;"
    coastline_result = api.query(coastline_request)
    node_request = "node(" + str(lat-0.005) + "," + str(lon-0.005) + "," + str(lat+0.005) + "," + str(lon+0.005) + ");out;"
    node_result = api.query(node_request)
    coastline_data = {}
    coastline_way_data = {}
    costline_node_data = {}
    f = open("coastline.yaml", "w")
    for coastline_way in coastline_result.ways:
        way_data = {}
        way_data["node_ids"] = coastline_way._node_ids
        coastline_way_data[coastline_way.id] = way_data
    for coastline_node in node_result.nodes:
        node_data = {}
        node_data["lat"] = float(coastline_node.lat)
        node_data["lon"] = float(coastline_node.lon)
        costline_node_data[coastline_node.id] = node_data
    coastline_data["way"] = coastline_way_data
    coastline_data["node"] = costline_node_data
    f.write(yaml.dump(coastline_data))
    f.close()
    
if __name__ == '__main__':
    fetch_result("coastline",21.31,-157.90)