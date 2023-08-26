#!/usr/bin/env python3
"""
Gets information about all IHMC rostopics
Emily Sheetz, Pathways Tour, Summer 2023

NOTE this script is inspired by the following resource:
https://answers.ros.org/question/256251/how-to-obtain-list-of-all-available-topics-python/
"""

import rostopic

output_file_name = "ihmc_rea_topic_info.txt"
topic_pattern = "ihmc/rea"

# get publisher and subscriber information for topics
pub_list, sub_list = rostopic.get_topic_list()

# create dictionary of topic information
topic_dict = {}

# store publisher information in dictionary
for topic in pub_list:
	# get topic information
	topic_name, msg_type, topic_pubs = topic
	# add to dictionary
	if topic_name not in topic_dict:
		topic_dict[topic_name] = {}
		topic_dict[topic_name]['msg_type'] = msg_type
		topic_dict[topic_name]['publishers'] = []
		topic_dict[topic_name]['subscribers'] = []
	topic_dict[topic_name]['publishers'] = topic_pubs

# store subscriber information in dictionary
for topic in sub_list:
	# get topic information
	topic_name, msg_type, topic_subs = topic
	# add to dictionary
	if topic_name not in topic_dict:
		topic_dict[topic_name] = {}
		topic_dict[topic_name]['msg_type'] = msg_type
		topic_dict[topic_name]['publishers'] = []
		topic_dict[topic_name]['subscribers'] = []
	topic_dict[topic_name]['subscribers'] = topic_subs

# get specifically IHMC topics
ihmc_topic_keys = [topic for topic in topic_dict.keys() if topic_pattern in topic]

# open output file
fo = open(output_file_name, 'w')

# write info to file
fo.write("Found " + str(len(ihmc_topic_keys)) + " topics with pattern: " + topic_pattern + "\n\n")

for topic_name in ihmc_topic_keys:
	# get info from dictionary
	msg_type = topic_dict[topic_name]['msg_type']
	topic_pub_list = topic_dict[topic_name]['publishers']
	topic_sub_list = topic_dict[topic_name]['subscribers']
	fo.write("Topic: " + topic_name + "\n")
	fo.write("\tType: " + msg_type + "\n")
	fo.write("\tPublishers:\n")
	if len(topic_pub_list) == 0:
		fo.write("\t\tNone\n")
	else:
		for pub in topic_pub_list:
			fo.write("\t\t" + pub + "\n")
	fo.write("\tSubscribers:\n")
	if len(topic_sub_list) == 0:
		fo.write("\t\tNone\n")
	else:
		for sub in topic_sub_list:
			fo.write("\t\t" + sub + "\n")
	fo.write("\n")

# close the file
fo.close()
