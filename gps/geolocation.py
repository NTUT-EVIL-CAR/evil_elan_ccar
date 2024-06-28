import requests

def get_current_location(api_key):
    url = 'https://www.googleapis.com/geolocation/v1/geolocate?key=' + api_key
    headers = {'Content-Type': 'application/json'}
    data = {'considerIp': 'true'}

    response = requests.post(url, headers=headers, json=data)

    if response.status_code == 200:
        result = response.json()
        location = result.get('location', {})
        latitude = location.get('lat')
        longitude = location.get('lng')
        accuracy = result.get('accuracy')
        print(f'Latitude: {latitude}, Longitude: {longitude}, Accuracy: {accuracy} meters')
    else:
        print(f'Error: {response.status_code}, {response.text}')

# 替换为您在 Google Cloud Console 中获得的 API 密钥
api_key = "AIzaSyBcUJ1dJgidqhqRnLJ8X2DMRdtNuzFFp-Q"

get_current_location(api_key)