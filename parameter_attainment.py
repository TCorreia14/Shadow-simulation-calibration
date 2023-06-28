import xml.etree.ElementTree as ET

PARAM_NAMES = ['stiffness', 'springref', 'frictionloss', 'damping', 'range'] #, 'range'

def isFloat(str): #Checks if all String is a numeric number
    try:
        float(str)
        return True
    except ValueError:
        return False

def firstCharacterStringCheck(str): #Checks if the first value of a String is an numeric Character
    if str[0].isdigit() or str[0]=="-":
        return True
    else:
        return False

def splitStringIntoFloat(str): #Divides String in several FLoats (Separeted by backspaces) -> Returns array of floats
    return list(map(float, str.split(" "))) 

def FLoatArrayIntoString(array):
    string = " ".join(str(x) for x in array)
    return string

def initTree(xml_path):
    tree = ET.parse(xml_path)
    root = tree.getroot()
    elements = root.findall('.//*')
    return tree, elements

def writeParam(tree, dest_path) :
    tree.write(dest_path)

def getParam(xml_path): #Gets all the parameters and returns an array of float with all the params
    
    tree = ET.parse(xml_path)
    root = tree.getroot()
    elements = root.findall('.//*')
    actual_param = [] 

    for element in elements:
        for attribute in element.attrib:
            actual_attrib = element.get(attribute)
            
            if isFloat(str(actual_attrib)):
                actual_param.append(float(str(actual_attrib)))
                
            elif firstCharacterStringCheck(str(actual_attrib)):
                actual_param.append(list(splitStringIntoFloat(actual_attrib))) 

    return actual_param

def setParam(xml_path, parameters): #Sets all the parameters (takes into argument array of floats with parameters)

    tree = ET.parse(xml_path)
    root = tree.getroot()
    elements = root.findall('.//*')
    i = 0
    
    for element in elements:
        for attribute in element.attrib:
            actual_attrib = element.get(attribute)
            
            if isFloat(actual_attrib):
                element.set(attribute, str(parameters[i]))
                i += 1  

            elif firstCharacterStringCheck(actual_attrib):
                element.set(attribute, str(FLoatArrayIntoString(parameters[i])))
                i += 1  

    writeParam(tree, xml_path)

def getParam_shared_options(xml_path): #Gets all the parameters and returns an array of float with all the params
    
    tree = ET.parse(xml_path)
    root = tree.getroot()
    elements = root.findall('.//*')
    actual_param = [] 

    # Find all elements that have attributes with the specified names
    for element in root.iter():
        for attrib_name in PARAM_NAMES:
            attrib_value = element.get(attrib_name)
            if attrib_value is not None and isFloat(attrib_value):
                actual_param.append(float(attrib_value))
            elif firstCharacterStringCheck(str(attrib_value)) and attrib_name =='range':
                actual_param.extend(list(splitStringIntoFloat(attrib_value)))
                
    # Return the extracted parameters
    return actual_param

def setParam_shared_options(xml_path, parameters): #Sets all the parameters (takes into argument array of floats with parameters)

    tree = ET.parse(xml_path)
    root = tree.getroot()

    i = 0
    
    for element in root.iter():
        for attrib_name in PARAM_NAMES:
            attrib_value = element.get(attrib_name)
            if attrib_value:
                if isFloat(attrib_value):
                    element.set(attrib_name, str(parameters[i]))
                    i += 1  
                elif firstCharacterStringCheck(attrib_value) and attrib_name =='range':
                    element.set(attrib_name, str(FLoatArrayIntoString([parameters[i],parameters[i+1]])))
                    i += 2  

    writeParam(tree, xml_path)

def getSingleParam_shared_options(xml_path, joint, thumb=False):
    
    tree = ET.parse(xml_path)
    root = tree.getroot()
    actual_param = [] 

    class_mujoco='finger'
    param_names = PARAM_NAMES


    if thumb:
        class_mujoco='thumb'
        param_names = ['frictionloss', 'damping']

    joint_element = root.find(f'.//default[@class="{class_mujoco}"]/default[@class="{joint}"]')

    # Find all elements that have attributes with the specified names
    for attrib_name in param_names:
        attrib_value = joint_element.find("./joint").get(attrib_name)
        if attrib_value is not None and isFloat(attrib_value):
            actual_param.append(float(attrib_value))
        elif firstCharacterStringCheck(str(attrib_value)) and attrib_name =='range':
            actual_param.extend(list(splitStringIntoFloat(attrib_value)))

    # Return the extracted parameters
    return actual_param



def setSingleParam_shared_options_joint_parameters(xml_path, parameters, joint, thumb=False): #Sets the parameters for a certain joint with only the input of parameters from that joint
    
    tree = ET.parse(xml_path)
    root = tree.getroot()

    class_mujoco='finger'
    param_names = PARAM_NAMES


    if thumb:
        class_mujoco='thumb'
        param_names = ['frictionloss', 'damping']

    joint_element = root.find(f'.//default[@class="{class_mujoco}"]/default[@class="{joint}"]/joint')
    if joint_element is not None:
        for i, attrib_name in enumerate(param_names):
            attrib_value = joint_element.get(attrib_name)
            if attrib_value is not None:
                if isFloat(attrib_value):
                    joint_element.set(attrib_name, str(parameters[i]))
                elif firstCharacterStringCheck(attrib_value) and attrib_name=='range':
                    joint_element.set(attrib_name, str(FLoatArrayIntoString([parameters[i],parameters[i+1]])))
                    i+=1
    
    writeParam(tree, xml_path)

def setSingleParam_shared_options(xml_path, parameters, joint): #Sets the parameters for a certain joint with the input of all the parameters from xml file

    tree = ET.parse(xml_path)
    root = tree.getroot()
    
    joint_element = root.find(f'.//default[@class="finger"]/default[@class="{joint}"]/joint')

    i = 0

    for element in root.iter():
        for attrib_name in PARAM_NAMES:
            attrib_value = element.get(attrib_name)
            if element.get('class') == joint:
                if (joint_element.attrib.get('range') is not None) and attrib_name == 'range':
                    joint_element.set(attrib_name, str(FLoatArrayIntoString([parameters[i],parameters[i+1]])))
                    i += 2
                elif attrib_name != 'range':
                    joint_element.set(attrib_name, str(parameters[i]))
                    i += 1
                else:
                    continue
            elif attrib_value and attrib_name=='range':
                i += 2
            elif attrib_value:
                i += 1
            else:
                continue

    writeParam(tree, xml_path)

if __name__ == "__main__":
    #params = getParam('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/sr_hand_e_plus_model.xml')
    #setParam('projects/shadow_robot/base/optimisationDTparams/rosbag_files/file_updated.xml', params)
    
    #params = getParam('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml')
    #setParam('projects/shadow_robot/base/optimisationDTparams/rosbag_files/file_updated.xml', params)
    
    params = getSingleParam_shared_options('/home/user/projects/shadow_robot/base/optimisationDTparams/extras/shared_options3.xml', 'THJ1', True)
    params[1]=params[1]+1 
    print(params)
    setSingleParam_shared_options_joint_parameters('/home/user/projects/shadow_robot/base/optimisationDTparams/extras/shared_options3.xml', params, 'THJ1', True)

