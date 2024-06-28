import geocoder

def get_location():
    location = geocoder.ip("me")
    return location.latlng

latitude, longitude = get_location()

print("Latitude:", latitude)
print("Longitude:", longitude)