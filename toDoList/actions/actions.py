
# This files contains your custom actions which can be used to run
# custom Python code.
#
# See this guide on how to implement these action:
# https://rasa.com/docs/rasa/custom-actions

from html import entities


from typing import Any, Text, Dict, List
#
from rasa_sdk import FormValidationAction, Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import SlotSet, ActiveLoop
import sqlite3
import os
from actions.utils import parse_date_time
project_home = os.environ['PROJECT_HOME']
import sys
sys.path.insert(0, project_home)
from create_table import connect
import datetime

import roslibpy

project_home = os.environ['PROJECT_HOME']
PATH_DB = project_home + '/toDoDB.db'

client=roslibpy.Ros(host='localhost', port=9090)
client.run()


class ValidateInsertForm(FormValidationAction):
# Validate slots for the insert form
    def name(self) -> Text:
        return "validate_insert_form"

    # set the alarm if present, else set False
    def validate_todo_alarm(self, slot_value: Any, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        if slot_value != False:
            if tracker.get_slot('todo_time') is None:
                return {'todo_alarm': True, 'todo_time':'00:00:00'}
            else:
                return {'todo_alarm': True}
        return {"todo_alarm": False}
    
    # extract tag from insert phrase
    async def extract_todo_tag(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict) -> Dict[Text, Any]:
        tag=''
        for value in tracker.get_latest_entity_values("tag"): # if tag is composed by more than one string
            tag += value+' '   
        if tag=='':
            return {}  
        return {"todo_tag": tag[0:-1]} # -1 remove the trailing ','
    
    
    # extract date if present 'time' slot present in phrase
    async def extract_todo_date(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict) -> Dict[Text, Any]:
        if tracker.latest_message['intent'].get('name')=='deny' and tracker.get_slot('todo_date') is None:
            return {'todo_date': 'null', 'todo_time': 'null', 'todo_alarm': False}
        if tracker.get_slot("time") is None:
            return {}
        return parse_date_time(tracker)
          
class ValidateDeleteForm(FormValidationAction):
# Validate slots for the delete form
    def name(self) -> Text:
        return "validate_delete_form"
    
    # extract tag from remove phrase
    async def extract_todo_tag(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict) -> Dict[Text, Any]:
        tag=''
        for value in tracker.get_latest_entity_values("tag"): # if tag is composed by more than one string
            tag += value+' '    
        if tag=='':
            return {}
        return {"todo_tag": tag[0:-1]} # -1 remove the trailing ','
       
    # Set the date and time if present
    async def extract_todo_date(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict) -> Dict[Text, Any]:
        if tracker.get_slot("time") is None:
            return {}
        return parse_date_time(tracker)

class ActionResetSlot(Action):
# Action to reset the slots
  def name(self) -> Text:
      return "action_reset_slot"

  async def run(
      self, dispatcher, tracker: Tracker, domain: Dict[Text, Any]
  ) -> List[Dict[Text, Any]]:
      # reset todo_tag, todo_category, todo_date, todo_time, todo_alarm
      return [SlotSet("time",None), SlotSet("todo_tag",None), SlotSet("todo_category",None), SlotSet("todo_date",None), SlotSet("todo_time",None), SlotSet("todo_alarm",None)]

class ActionInsertToDoDB(Action):
# Action that executes when inserting to-do into the database
    def name(self) -> Text:
      return "action_insert_db"

    def run(
        self, dispatcher, tracker: Tracker, domain: Dict[Text, Any]
    ) -> List[Dict[Text, Any]]:
        if tracker.get_slot('user_id') is None: # if user was not authenticated
            dispatcher.utter_message("Please, identify yourself first")
            return [ActiveLoop(None)]
        try:     
            sqliteConnection = sqlite3.connect(PATH_DB)
            cursor = sqliteConnection.cursor()
            print("Successfully Connected to SQLite")
            # Query to insert into the todo table
            sqlite_insert_query = """INSERT INTO todo
                            (tag, date, time, alarm, user_id, category_name) 
                            VALUES 
                            (?, ?, ?, ?, ?, ?)"""
            tag=tracker.get_slot("todo_tag")
            cursor.execute(sqlite_insert_query, (tag, tracker.get_slot("todo_date"), tracker.get_slot("todo_time"), 1 if tracker.get_slot("todo_alarm") else 0, tracker.get_slot('user_id'), tracker.get_slot('todo_category'), ))
            sqliteConnection.commit()
            print("Record inserted successfully into todo table ", cursor.rowcount)
            cursor.close()
            dispatcher.utter_message(response='utter_insert_finished')
        except sqlite3.IntegrityError as error:
            dispatcher.utter_message(response='utter_insert_not_performed')
        except sqlite3.Error as error:
            print("Failed ", error)
        finally:
            if sqliteConnection:
                sqliteConnection.close()
                print("The SQLite connection is closed")
                
                
class ActionDeleteToDoDB(Action):
    # Action that executes when the user wants to delete a todo
    def name(self) -> Text:
      return "action_delete_db"

    def run(
        self, dispatcher, tracker: Tracker, domain: Dict[Text, Any]
    ) -> List[Dict[Text, Any]]:
        # check if the user is authenticated
        if tracker.get_slot('user_id') is None:
            dispatcher.utter_message("Please, identify yourself first")
            return [ActiveLoop(None)]
        try:
            sqliteConnection = sqlite3.connect(PATH_DB)
            cursor = sqliteConnection.cursor()
            print("Successfully Connected to SQLite")
            # Delete a todo if tag, date and time are provided
            if tracker.get_slot("todo_tag") is not None:
                sqlite_delete_query = """DELETE FROM todo WHERE tag=? AND user_id=?"""
                cursor.execute(sqlite_delete_query, (tracker.get_slot("todo_tag"), (tracker.get_slot("user_id")), ))
                sqliteConnection.commit()
                if cursor.rowcount==1:
                    # to-do found     
                    dispatcher.utter_message(response='utter_delete_finished')
                    print("Elimination successfully performed")
                else:
                    # no to-do found
                    dispatcher.utter_message(response='utter_delete_not_performed') 
                    print("Elimination NOT performed")
            else:
                # not all field provided, do nothing
                return []
            cursor.close()
        except sqlite3.Error as error:
            print("Failed ", error)
        finally:
            if sqliteConnection:
                sqliteConnection.close()
                print("The SQLite connection is closed")

class ActionVisualizeToDoDB(Action):
# Action to show the to-do list
    def name(self) -> Text:
      return "action_visualize_db"

    def run(
        self, dispatcher, tracker: Tracker, domain: Dict[Text, Any]
    ) -> List[Dict[Text, Any]]:
        # check if the user is authenticated
        if tracker.get_slot('user_id') is None:
            dispatcher.utter_message("Please, identify yourself first")
            return [ActiveLoop(None)]
            
        # get category and date-time if present
        category = next(tracker.get_latest_entity_values('category'), None)
        time = next(tracker.get_latest_entity_values('time'), None)
        info_message=""
        if time is not None and category is not None: #if time and category is present, get the to-dos only with the date and category selected
            date = parse_date_time(tracker)['todo_date']
            info_message={'id': str(tracker.sender_id), 'category': category, 'date': date}
        elif time is not None: #if only time is present, get only the date and set the select query
            date = parse_date_time(tracker)['todo_date']
            info_message={'id': str(tracker.sender_id), 'date': date}
        elif category is not None: #if only category is present, get only the category and set the select query
            info_message={'id': str(tracker.sender_id), 'category': category}
        else: # set the select query
            info_message={'id': str(tracker.sender_id)}

        talker = roslibpy.Topic(client, '/info_topic', 'pepper_nodes/InfoView')
        if client.is_connected:
            talker.publish(roslibpy.Message(info_message))
            print('Sending message...')
        else:
            print("error")
        dispatcher.utter_message("Here there are your to-dos!")
    
class ActionUserToDoDB(Action):
    # Action that executes every time the user wants to authenticate
    def name(self) -> Text:
      return "action_user_db"

    def run(
        self, dispatcher, tracker: Tracker, domain: Dict[Text, Any]
    ) -> List[Dict[Text, Any]]:
        try:
            sqliteConnection = connect()
            cursor = sqliteConnection.cursor()
            print("Successfully Connected to SQLite")
            # Get the name of the user with sender_id (provided from ROS)
            sqlite_search_query = """SELECT name FROM user WHERE id=?"""
            cursor.execute(sqlite_search_query, (str(tracker.sender_id),)) 

            rows = cursor.fetchall()            
            if tracker.get_slot("name") is None:
                dispatcher.utter_message("I didn't understand your name. Please rephrase!")
                return []
            if len(rows)==1:
                # user found
                if rows[0][0] is None:
                    # first time of user presenting to the robot
                    sqlite_update_query = """UPDATE user SET name=? WHERE id=?"""
                    cursor.execute(sqlite_update_query, (str(tracker.get_slot("name")).capitalize(), tracker.sender_id,))

                    sqliteConnection.commit()
                    dispatcher.utter_message("Welcome " + str(tracker.get_slot("name")).capitalize() + ", I added you")

                    return [SlotSet('user_id', str(tracker.sender_id)), SlotSet('name', str(tracker.get_slot("name")).capitalize())]

                elif rows[0][0]==str(tracker.get_slot("name")).capitalize():
                    # name provided by the user match with the database
                    dispatcher.utter_message("Hello " + str(tracker.get_slot("name")).capitalize())
                    return [SlotSet('user_id', str(tracker.sender_id)), SlotSet('name', str(tracker.get_slot("name")).capitalize())]
                else:
                    # name do not match, authentication denied
                    dispatcher.utter_message("Authentication failed. I think you are " + str(rows[0][0]))
            else:
                dispatcher.utter_message("Error recognition")
                # communicate to ROS that the user is not present
                return [SlotSet('user_id', str(-1)), SlotSet('name', str(tracker.get_slot("name")).capitalize())]
            
            cursor.close()
        except sqlite3.Error as error:
            print("Failed ", error)
        finally:
            if sqliteConnection:
                sqliteConnection.close()
                print("The SQLite connection is closed")
                
class ActionDeleteAllToDoDB(Action):
    # Action that executes when the user wants to delete all the to-do in their list
    def name(self) -> Text:
      return "action_delete_all_db"

    def run(
        self, dispatcher, tracker: Tracker, domain: Dict[Text, Any]
    ) -> List[Dict[Text, Any]]:
        
        # check if the user is identified
        if tracker.get_slot('user_id') is None:
            dispatcher.utter_message("Please, identify yourself first")
            return [ActiveLoop(None)]
        
        try:
            sqliteConnection = sqlite3.connect(PATH_DB)
            cursor = sqliteConnection.cursor()
            print("Successfully Connected to SQLite")
            
            # query to delete all items in the todo table for the registered user
            sqlite_delete_all_query = """DELETE FROM todo where user_id=?"""
            cursor.execute(sqlite_delete_all_query, (tracker.get_slot('user_id'),))
            sqliteConnection.commit()
            if cursor.rowcount>0:
                # deleted 1 or more to-dos   
                dispatcher.utter_message('I successfully deleted ' + str(cursor.rowcount) + ' reminders')
                print("Elimination successfully performed")
            else:
                # did not find any to-do to delete
                dispatcher.utter_message('I could not find any reminder') 
                print("Elimination NOT performed")
            cursor.close()

        except sqlite3.Error as error:
            print("Failed ", error)
        finally:
            if sqliteConnection:
                sqliteConnection.close()
                print("The SQLite connection is closed")
               

class ValidateUpdateForm(FormValidationAction):
# Validate slots for the update form
    def name(self) -> Text:
        return "validate_update_form"

    # set the alarm if present, else set False, also set other slots to 'null'
    async def extract_todo_alarm(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict) -> Dict[Text, Any]:
        if tracker.get_slot('todo_tag') is None or len(list(tracker.get_latest_entity_values("alarm"))) == 0:
            return {}
        try:
            sqliteConnection = sqlite3.connect(PATH_DB)
            cursor = sqliteConnection.cursor()
            print("Successfully Connected to SQLite")
            
            # Get the name of the user with sender_id (provided from ROS)
            sqlite_search_query = """SELECT alarm, time, date FROM todo WHERE tag=? AND user_id=?"""
            cursor.execute(sqlite_search_query, (tracker.get_slot('todo_tag'),tracker.get_slot('user_id'),))
            rows = cursor.fetchall()
            d = {}
            if len(rows)==0:
                # todo not found
                dispatcher.utter_message("I can't find this tag. Try to repeat!")
                return {}
            elif rows[0][0] == 0:
                # alarm was not set, also sets date and time if not present
                d['todo_alarm'] = True
                d['todo_category'] = 'null'
                if rows[0][1] == 'null':
                    d['todo_time'] = '00:00:00'
                if rows[0][2] == 'null':
                    date = datetime.date.today() + datetime.timedelta(days=1)
                    d['todo_date'] = date.strftime('%Y-%m-%d')
                return d
            else:
                # alarm was set
                return {'todo_alarm':False, 'todo_category': 'null', 'todo_date':'null', 'todo_time':'null'}
        except sqlite3.Error as error:
            print("Failed ", error)
        finally:
            if sqliteConnection:
                sqliteConnection.close()
                print("The SQLite connection is closed")

    # extract tag from insert phrase
    async def extract_todo_tag(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict) -> Dict[Text, Any]:
        tag=''
        for value in tracker.get_latest_entity_values("tag"): # if tag is composed by more than one string
            tag += value+' ' 
        if tag=='':
            return {}
        return {"todo_tag": tag[0:-1]} # -1 remove the trailing ','
    
    
    # extract date if 'time' slot present in phrase, also set other fields to 'null'
    async def extract_todo_date(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict) -> Dict[Text, Any]:
        if tracker.get_slot("time") is None:
            return {}
        d = parse_date_time(tracker)
        d['todo_category'] = 'null'
        d['todo_alarm'] = 'null'
        if d.get('todo_date', None) is None:
            d['todo_date'] = 'null'
        if d.get('todo_time', None) is None:
            d['todo_time'] = 'null'
        return d

    # extract date if 'category' slot present in phrase, also set other fields to 'null'
    async def extract_todo_category(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict) -> Dict[Text, Any]:
        print('Todo category')
        for entity in tracker.latest_message['entities']:
            if entity['entity'] == 'category':
                return {'todo_category': entity['value'], 'todo_date':'null', 'todo_time':'null', 'todo_alarm':'null'}
        return {}


class ActionUpdateToDoDB(Action):
    # Action that executes when the user wants to update a to-do in their list
    def name(self) -> Text:
      return "action_update_db"

    def run(
        self, dispatcher, tracker: Tracker, domain: Dict[Text, Any]
    ) -> List[Dict[Text, Any]]:
        # check if the user is identified
        if tracker.get_slot('user_id') is None:
            dispatcher.utter_message("Please, identify yourself first")
            return [ActiveLoop(None)]
        
        try:
            sqliteConnection = sqlite3.connect(PATH_DB)
            cursor = sqliteConnection.cursor()
            print("Successfully Connected to SQLite")
            
            if tracker.get_slot('todo_category') != 'null':
                # query to delete all items in the todo table for the registered user
                sqlite_update_query = """UPDATE todo SET category_name=? WHERE user_id=? AND tag=?"""
                cursor.execute(sqlite_update_query, (tracker.get_slot('todo_category'),tracker.get_slot('user_id'),tracker.get_slot('todo_tag'),))
            if tracker.get_slot('todo_date') != 'null':
                # query to delete all items in the todo table for the registered user
                sqlite_update_query = """UPDATE todo SET date=? WHERE user_id=? AND tag=?"""
                cursor.execute(sqlite_update_query, (tracker.get_slot('todo_date'),tracker.get_slot('user_id'),tracker.get_slot('todo_tag'),))
            if tracker.get_slot('todo_time') != 'null':
                # query to delete all items in the todo table for the registered user
                sqlite_update_query = """UPDATE todo SET time=? WHERE user_id=? AND tag=?"""
                cursor.execute(sqlite_update_query, (tracker.get_slot('todo_time'),tracker.get_slot('user_id'),tracker.get_slot('todo_tag'),))
            if tracker.get_slot('todo_alarm') != 'null':
                # query to delete all items in the todo table for the registered user
                sqlite_update_query = """UPDATE todo SET alarm=? WHERE user_id=? AND tag=?"""
                cursor.execute(sqlite_update_query, (1 if tracker.get_slot("todo_alarm") else 0,tracker.get_slot('user_id'),tracker.get_slot('todo_tag'),))
            
            sqliteConnection.commit()

            if cursor.rowcount>0:
                # deleted 1 or more to-dos   
                dispatcher.utter_message('I successfully update your reminder')
                print("Update successfully performed")
            else:
                # did not find any to-do to delete
                dispatcher.utter_message('I could not update your reminder') 
                print("Update NOT performed")
            cursor.close()

        except sqlite3.Error as error:
            print("Failed ", error)
        finally:
            if sqliteConnection:
                sqliteConnection.close()
                print("The SQLite connection is closed")