"""Copyright 2023 Brookhaven National Laboratory BSD 3 Clause License. See LICENSE.txt for details."""

import redis

# Connect to Redis
# Note: Change the host IP to map your server.
# Following command can be used to retrieve the host address of a redis server
# podman container inspect -f '{{.NetworkSettings.IPAddress}}' redis-container
client = redis.Redis(host="192.168.56.1", port=6379, db=0)

# Step 1: Store tag data in Redis
client.hset("tag:1", mapping={"id": 0, "family": "DICT_APRILTAG_36h11", "size": 0.02665, "sample_name": "sample_1"})
client.hset("tag:2", mapping={"id": 2, "family": "DICT_APRILTAG_36h12", "size": 0.02665, "sample_name": "sample_2"})
client.hset("tag:3", mapping={"id": 3, "family": "DICT_APRILTAG_36h12", "size": 0.02665, "sample_name": "sample_3"})
client.hset("tag:4", mapping={"id": 4, "family": "DICT_APRILTAG_36h12", "size": 0.02665, "sample_name": "sample_4"})
client.hset("tag:5", mapping={"id": 5, "family": "DICT_APRILTAG_36h12", "size": 0.02665, "sample_name": "sample_5"})
client.hset("tag:6", mapping={"id": 6, "family": "DICT_APRILTAG_36h12", "size": 0.02665, "sample_name": "sample_6"})

# Step 2: Indexing the sample_name to the tag key (e.g., tag:1, tag:2)
client.hset("sample_name_index", "sample_1", "tag:1")
client.hset("sample_name_index", "sample_2", "tag:2")
client.hset("sample_name_index", "sample_3", "tag:3")
client.hset("sample_name_index", "sample_4", "tag:4")
client.hset("sample_name_index", "sample_5", "tag:5")
client.hset("sample_name_index", "sample_6", "tag:6")
