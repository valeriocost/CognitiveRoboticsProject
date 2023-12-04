#!/bin/bash

cd $PROJECT_HOME/toDoList
source venv/bin/activate
rasa run -m models --endpoints endpoints.yml --port 5002 --credentials credentials.yml --enable-api
