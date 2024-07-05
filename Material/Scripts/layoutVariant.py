import xml.etree.ElementTree as ET
import utils
import os

# Function to create XML files with varying BotCount values
def create_variant_files():
    #mode = input('Pick an variant: (1) bots. (2) stations')
    files, folder = utils.askFiles("*.xlayo");
    for filePath in files:
        print(f"Generating variant of: {filePath}")
        #filePath = input("Enter the file (.xlayo): ").strip('"')
        directory, filename = os.path.split(filePath)
        name, extension = os.path.splitext(filename)
        # Read the original XML file
        tree = ET.parse(filePath)
        root = tree.getroot()
        
        for bot_count in range(10, 110, 10):
            # Modify the BotCount value
            for bot in root.findall('BotCount'):
                bot.text = str(bot_count)
            # Modify the Name
            for nameLayout in root.findall('NameLayout'):
                nameLayout.text = str(f"{name}r{bot_count}")
            
            # Create a file with the name including the current BotCount
            new_file_name = f"{name}r{bot_count}{extension}"
            new_file_path = os.path.join(directory, new_file_name)
            
            # Write the modified XML data to the new file
            tree.write(new_file_path, encoding='utf-8', xml_declaration=True)

# Call the function to create the files
create_variant_files()
