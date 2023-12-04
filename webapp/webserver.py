from flask import Flask, render_template, request
import sqlite3
import os
from datetime import datetime, date

app = Flask(__name__)


@app.route("/view", methods=['GET', 'POST'])
def view():
    project_home = os.environ['PROJECT_HOME']
    sqliteConnection = sqlite3.connect(project_home + "/toDoDB.db")
    cursor = sqliteConnection.cursor()
    cursor_alarm = sqliteConnection.cursor()
    print("Successfully Connected to SQLite")
    
    id=request.args.get('id')
    category=request.args.get('category')
    date=request.args.get('date')

    if category is not None and date is not None:
        sqlite_search_query = "SELECT tag,date,time,alarm, category_name, user_id FROM todo WHERE date=? AND category_name=? AND user_id=? ORDER BY date, time ASC"
        cursor.execute(sqlite_search_query, (date, category, id, ))
    elif date is not None:
        sqlite_search_query = "SELECT tag,date,time,alarm, category_name, user_id FROM todo WHERE date=? AND user_id=? ORDER BY date, time ASC"
        cursor.execute(sqlite_search_query, (date, id,))
    elif category is not None:
        sqlite_search_query = "SELECT tag,date,time,alarm, category_name, user_id FROM todo WHERE category_name=? AND user_id=? ORDER BY date, time ASC"
        cursor.execute(sqlite_search_query, (category, id, ))
    else:
        sqlite_search_query = "SELECT tag,date,time,alarm,category_name, user_id FROM todo WHERE user_id=? ORDER BY date, time ASC"
        cursor.execute(sqlite_search_query, (id,))
    
    sqlite_alarm_query = "select tag,date,time,alarm,category_name, user.name, user_id FROM todo, user WHERE todo.user_id = user.id AND alarm = 1 AND date = DATE('now') and time BETWEEN TIME('now', 'localtime') and TIME('now', '+30 minutes', 'localtime')"
    cursor_alarm.execute(sqlite_alarm_query)
    data_alarm = cursor_alarm.fetchall()
    data = cursor.fetchall()

    return render_template('view.html', data=data, data_alarm=data_alarm, id=id)

@app.template_filter()
def check_alarm(a, b):
    for row in b:
        if a[0] == row[0] and a[-1] == row[-1]:
            return True
    return False

@app.template_filter()
def check_date(a):
    if a[1] == 'null' or a[2] == 'null':
        return False
    datetime_string = a[1] + ' ' + a[2]
    todo_date = datetime.strptime(datetime_string, '%Y-%m-%d %H:%M:%S')
    now = datetime.now()
    if todo_date < now:
        return True
    return False

@app.template_filter()
def return_names(a):
    names = list(set([row[5] for row in a]))
    return ", ".join(names)

if __name__== "__main__":
    app.static_folder = 'static'
    app.run(host="0.0.0.0")
