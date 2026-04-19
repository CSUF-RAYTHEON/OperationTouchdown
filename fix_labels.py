import os

mapping = {
    0: 0,  # bucket -> bucket
    1: 0,  # buckets -> bucket
    2: 2,  # cardboard -> box
    3: 1,  # cones -> cone
    4: 3   # ramp -> ramp
}

for split in ["train", "val"]:
    label_dir = os.path.join(split, "labels")

    if not os.path.exists(label_dir):
        print(f"Missing folder: {label_dir}")
        continue

    for filename in os.listdir(label_dir):
        if not filename.endswith(".txt"):
            continue

        path = os.path.join(label_dir, filename)
        new_lines = []

        with open(path, "r") as f:
            for line in f:
                parts = line.strip().split()
                if not parts:
                    continue

                old_class = int(parts[0])

                if old_class not in mapping:
                    continue

                new_class = mapping[old_class]
                parts[0] = str(new_class)
                new_lines.append(" ".join(parts))

        with open(path, "w") as f:
            f.write("\n".join(new_lines))

print("Done fixing labels.")