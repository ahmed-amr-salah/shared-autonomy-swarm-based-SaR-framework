import sys
import os
sys.path.append("..")
import utilitiess.templates as templates
import utilitiess.parse as parser

current_path = os.path.dirname(os.path.realpath(__file__))
new_path = os.path.join(current_path, '..', '..')

class Database:
    def __init__(self, node_info):
        self.node_info = node_info
        file_path = os.path.join(new_path, 'AUC-Thesis-Leithy-DT/RemoteDrivingDashboard-master/src/local_cloud', f"{self.node_info.node_name}_database_template.py")
        with open(file_path, "w") as file:
            writer = templates.FileWriterr(file=file)
            file.write("""#___python lib__
from django.shortcuts import resolve_url
import requests
import json
from datetime import datetime
import mysql.connector
import random
#___src___
from django.db import connections\n\n""")
            writer.establish_DB_connection()
            writer.write_table_create(node_name = self.node_info.node_name + "_table")
            writer.write_table_insert(node_name = self.node_info.node_name + "_table")
            writer.write_table_search()
            