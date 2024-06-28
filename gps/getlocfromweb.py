# from selenium import webdriver 
# from selenium.webdriver.chrome.options import Options 
# from selenium.webdriver.support.ui import WebDriverWait
# import time

# def getLocation():
#     options = Options()
#     options.add_argument("--use--fake-ui-for-media-stream")
#     driver = webdriver.Chrome(options=options)

#     timeout = 20
#     driver.get("https://mycurrentlocation.net/")
#     wait = WebDriverWait(driver, timeout)
#     time.sleep(10)

#     longitude = driver.find_elements_by_xpath('//*[@id="longitude"]') #Replace with any XPath    
#     longitude = [x.text for x in longitude]    
#     longitude = str(longitude[0])    
#     latitude = driver.find_elements_by_xpath('//*[@id="latitude"]')    
#     latitude = [x.text for x in latitude]    
#     latitude = str(latitude[0])    
#     driver.quit()    
#     return (latitude,longitude)

# print(getLocation())

from selenium import webdriver
from selenium.webdriver.chrome.options import Options

def get_location_from_website():
    options = Options()
    options.add_argument("--use-fake-ui-for-media-stream")

    # 如果您的 chromedriver 不在当前目录，请指定其路径
    driver = webdriver.Chrome(options=options)

    try:
        # 打开网站
        driver.get("https://mycurrentlocation.net/")

        # 等待页面加载
        driver.implicitly_wait(20)

        while True:

            longitude_element = driver.find_element("#detail-longitude")
            latitude_element = driver.find_element("#detail-latitude")
            driver.find_element

            longitude = longitude_element.text
            latitude = latitude_element.text

            print(f'Latitude: {latitude}, Longitude: {longitude}')

    except Exception as e:
        print(f'Error: {e}')

    finally:
        # 关闭浏览器
        driver.quit()

if __name__ == "__main__":
    get_location_from_website()
