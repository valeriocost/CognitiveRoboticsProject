from datetime import datetime, timedelta

def parse_date_time(tracker):
    entities=tracker.latest_message['entities']
    add_info=None
    for x in entities:
        if (x['entity']=='time'):
            add_info=x['additional_info']
            break
    if add_info is None:
        return {}
    print(str(add_info))
    f = "%Y-%m-%dT%H:%M:%S"
    if add_info is None:
        return {'time': None}
    if add_info['type']=='interval':
        print(str(add_info['from']['value']))
        dt = datetime.strptime(str(add_info['to']['value'])[0:-10], f) - timedelta(hours=4)
        return {'time': None, 'todo_date': dt.strftime('%Y-%m-%d'), 'todo_time': dt.strftime('%H:%M:%S')}  
    elif add_info['grain'] in ['hour', 'minute', 'second']:
            dt = datetime.strptime(str(add_info['value'])[0:-10], f)
            if tracker is not None and tracker.get_slot('todo_date') is not None:
                return {'time': None, 'todo_time': dt.strftime('%H:%M:%S')}
            return {'time': None, 'todo_date': dt.strftime('%Y-%m-%d'), 'todo_time': dt.strftime('%H:%M:%S')}        
    else:
        dt = datetime.strptime(str(add_info['value'])[0:-10], f)
        return {'time': None, 'todo_date': dt.strftime('%Y-%m-%d'), 'todo_time':'00:00:00'} 