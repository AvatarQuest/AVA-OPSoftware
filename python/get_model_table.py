import requests
from bs4 import BeautifulSoup
import re
import json

output = {"eeprom": {}, "ram": {}}
row_names = []

res = requests.get(
    "https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/")

soup = BeautifulSoup(res.content, 'html.parser')
eeprom_div = soup.find('h2', {'id': "control-table-of-eeprom-area"})
eeprom_table = eeprom_div.findNext('table')

head = eeprom_table.find('thead').find('tr')
row_names = [name.text.lower() for name in head.find_all('th')]

print(row_names)

for row in eeprom_table.find('tbody').find_all('tr'):
    row_name = row.find_all('td')[2].text.lower()
    row_value = row.find_all('td')[0].text

    try:
        row_value = int(row_value)
    except:
        pass

    print(row_name)
    print(row_value)

    output["eeprom"].__setitem__(row_name, row_value)


ram_div = soup.find('h2', {'id': "control-table-of-ram-area"})
ram_table = ram_div.findNext('table')

head = ram_table.find('thead').find('tr')
row_names = [name.text.lower() for name in head.find_all('th')]

print(row_names)
last_tag = None
indirect = False

for row in ram_table.find('tbody').find_all('tr'):
    row_name = row.find_all('td')[2].text.lower()
    row_value = row.find_all('td')[0].text

    if "Indirect" in row_name:
        continue

    try:
        row_value = int(row_value)
    except:
        pass

    print(row_name)
    print(row_value)

    output["ram"].__setitem__(row_name, row_value)

    last_tag = {"name": row_name, "value": row_value}

# last_tag = None
# data_section = 1
# address_section = 1
# address_start = 0
# data_start = 0
# json = {}
# address = False
# data = False

# for row in ram_table.find('tbody').find_all('tr'):
#     row_name = row.find_all('td')[2].text.lower()

#     if not "Indirect" in row_name:
#         continue

#     if "Address" in row_name:
#         if data:
#             byte_size = last_tag.find_all('td')[1].text.lower()
#             json = {"byte size": byte_size, "begin": {"idx": None,
#                                                       "addr": None}, "end": {"idx": None, "addr": None}}
#             output["ram"].__setitem__(
#                 f"indirect address section {address_section}", json)
#             address_section += 1

#         address_start = int(re.findall(r'\d+', row_name)[0])
#         address = True

#     elif "Data" in row_name:
#         data = True

#     row_value = row.find_all('td')[0].text

#     last_tag = row

with open('output.json', 'w+') as file:
    file.write(json.dumps(output))

print(output)
