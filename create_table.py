import sqlite3
import numpy as np
import io
import os

project_home = os.environ['PROJECT_HOME']
PATH_DB = project_home + '/toDoDB.db'

def connect():
    def adapt_array(arr):
        out = io.BytesIO()
        np.save(out, arr)
        out.seek(0)
        return sqlite3.Binary(out.read())

    def convert_array(text):
        out = io.BytesIO(text)
        out.seek(0)
        return np.load(out)


    # Converts np.array to TEXT when inserting
    sqlite3.register_adapter(np.ndarray, adapt_array)

    # Converts TEXT to np.array when selecting
    sqlite3.register_converter("array", convert_array)


    con = sqlite3.connect(PATH_DB, detect_types=sqlite3.PARSE_DECLTYPES)
    return con

if __name__ == '__main__':
    con = connect()
    # cur = con.cursor()
    # cur.execute("""CREATE TABLE "user" (
    #     "id"	INTEGER NOT NULL,
    #     "name"	TEXT,
    #     "feature_vector" array,
    #     PRIMARY KEY("id" AUTOINCREMENT)
    # );""")
