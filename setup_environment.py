"""
Helper script to generate environment-specific XML files.
Usage: python setup_environment.py [normal|sand]
"""
import sys
import xml.etree.ElementTree as ET

def set_ground_type(input_xml, output_xml, ground_type):
    """Modify the ground material based on environment type."""
    tree = ET.parse(input_xml)
    root = tree.getroot()
    
    # Find the ground geom
    worldbody = root.find('worldbody')
    if worldbody is None:
        print("Error: No worldbody found")
        return False
    
    ground_geom = worldbody.find(".//geom[@name='ground']")
    if ground_geom is None:
        print("Error: No ground geom found")
        return False
    
    # Set material and friction based on ground type
    if ground_type == "sand":
        ground_geom.set('material', 'matsand')
        ground_geom.set('friction', '2.5 0.05 0.01')  # Lower friction, higher rolling/torsional for sand
        print("✓ Set environment to SAND (lower friction, grainy)")
    else:  # normal/default
        ground_geom.set('material', 'matplane')
        ground_geom.set('friction', '6.0 0.01 0.002')  # High friction for hard surface
        print("✓ Set environment to NORMAL (checker pattern, high friction)")
    
    # Write output
    tree.write(output_xml, encoding='unicode', xml_declaration=True)
    return True

if __name__ == "__main__":
    ground_type = sys.argv[1] if len(sys.argv) > 1 else "normal"
    
    if ground_type not in ["normal", "sand"]:
        print(f"Unknown ground type: {ground_type}")
        print("Usage: python setup_environment.py [normal|sand]")
        sys.exit(1)
    
    success = set_ground_type(
        "snake_simple.xml",
        "snake_simple_env.xml", 
        ground_type
    )
    
    if success:
        print(f"Generated: snake_simple_env.xml")
    else:
        sys.exit(1)

