import re
import sys

# Factors
SCALE_FACTOR = 2.0
MASS_FACTOR = 9.0  # To go from ~5.5kg to ~50kg, and consistent with Volume scale (2^3=8)
INERTIA_FACTOR = MASS_FACTOR * (SCALE_FACTOR ** 2) # I ~ M * L^2

def scale_numbers(text, factor):
    def replace(match):
        val = float(match.group(0))
        return f"{val * factor:.6g}"
    return re.sub(r'-?\d+(\.\d+)?([eE][+-]?\d+)?', replace, text)

def scale_xyz(xyz_str):
    nums = []
    for x in xyz_str.split():
        try:
            nums.append(float(x))
        except ValueError:
            return xyz_str # Return original if it contains variables like ${...}
    scaled = [n * SCALE_FACTOR for n in nums]
    return f"{scaled[0]:.6g} {scaled[1]:.6g} {scaled[2]:.6g}"

def process_file(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Scale Origins
    def replace_origin(match):
        full_tag = match.group(0)
        if 'xyz="' in full_tag:
            xyz_match = re.search(r'xyz="([^"]+)"', full_tag)
            if xyz_match:
                original_xyz = xyz_match.group(1)
                new_xyz = scale_xyz(original_xyz)
                return full_tag.replace(original_xyz, new_xyz)
        return full_tag
    
    content = re.sub(r'<origin[^>]*>', replace_origin, content)

    # Scale Mass
    def replace_mass(match):
        val = float(match.group(1))
        return f'<mass value="{val * MASS_FACTOR:.6g}"/>'
    content = re.sub(r'<mass value="([^"]+)"/>', replace_mass, content)

    # Scale Inertia
    def replace_inertia(match):
        attrs = match.group(0)
        def scale_attr(m):
            v = float(m.group(1))
            return f'="{v * INERTIA_FACTOR:.6g}"'
        return re.sub(r'="(-?\d+(\.\d+)?([eE][+-]?\d+)?)"', scale_attr, attrs)
    content = re.sub(r'<inertia[^>]*/>', replace_inertia, content)

    # Scale Cylinder Geometry
    def replace_cylinder(match):
        tag = match.group(0)
        def scale_val(m):
            v = float(m.group(1))
            return f'="{v * SCALE_FACTOR:.6g}"'
        tag = re.sub(r'radius="([^"]+)"', lambda m: f'radius="{float(m.group(1))*SCALE_FACTOR:.6g}"', tag)
        tag = re.sub(r'length="([^"]+)"', lambda m: f'length="{float(m.group(1))*SCALE_FACTOR:.6g}"', tag)
        return tag
    content = re.sub(r'<cylinder[^>]*/>', replace_cylinder, content)

    # Add Scale to Meshes
    def replace_mesh(match):
        tag = match.group(0)
        if 'scale=' not in tag:
            return tag.replace('/>', f' scale="{SCALE_FACTOR} {SCALE_FACTOR} {SCALE_FACTOR}"/>')
        return tag
    content = re.sub(r'<mesh filename="[^"]+"/>', replace_mesh, content)

    # Specific replacements for DiffDrive and specific parameters
    content = content.replace('0.358', str(0.358 * SCALE_FACTOR)) # wheel_separation
    content = content.replace('0.0625', str(0.0625 * SCALE_FACTOR)) # wheel_radius
    
    # Scale imu_translationDefault
    # imu_translation:='0.0628 -0.0314 -0.0393'''
    def replace_imu_default(match):
        xyz = match.group(1)
        return f"imu_translation:='{scale_xyz(xyz)}'"
    content = re.sub(r"imu_translation:='([^']+)'", replace_imu_default, content)

    with open(filepath, 'w') as f:
        f.write(content)

if __name__ == '__main__':
    process_file(sys.argv[1])
