import json 

def change_unsafe(input_fn, output_fn, factor = 1):
    f = open(input_fn, 'r')
    scene = json.load(f)
    f.close()
    unsafe = scene['unsafeSet']
    for i in range(len(unsafe)):
        unsafe_box = unsafe[i][1]
        for j in range(len(unsafe_box[0])):
            unsafe_box[0][j] += factor
            unsafe_box[1][j] -= factor
    f = open(output_fn,'w+')
    scene = json.dump(scene, f)
    f.close()
    pass 

if __name__ == "__main__":
    input_fn = './scene_complex2_v2.json'
    output_fn = './scene_complex2_v3.json'
    change_unsafe(input_fn, output_fn)