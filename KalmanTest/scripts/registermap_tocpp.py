import csv

with open('./docs/register_map.csv', newline='', encoding="UTF-8") as csvfile:
    reader = csv.reader(csvfile, delimiter=';', quotechar='|')
    for row in reader:
        print(f"#define REG_{row[2]} 0x{row[0]}")
