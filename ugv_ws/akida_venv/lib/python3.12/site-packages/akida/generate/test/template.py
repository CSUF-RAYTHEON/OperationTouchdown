import os


def test_file_from_template(test_name, tpl_path, dest_path, filename):
    src_path = os.path.join(tpl_path, filename)
    dst_path = os.path.join(dest_path, test_name, filename)
    with open(src_path, 'r') as src, open(dst_path, 'w') as dst:
        for line in src:
            # Substitute pattern by filename
            dst.write(line.replace("TEST_NAME", test_name))
