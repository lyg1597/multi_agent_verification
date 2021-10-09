import json 

def change_unsafe(fn, factor = 1):
    f = open(fn, 'r')
    scene = json.load(f)
    f.close()
    unsafe = scene['unsafeSet']
    for i in range(len(unsafe)):
        unsafe_box = unsafe[i][1]
        for j in range(len(unsafe_box[0])):
            unsafe_box[0][j] += factor
            unsafe_box[1][j] -= factor
    f = open(fn,'w+')
    scene = json.dump(scene, f)
    f.close()
    pass 

if __name__ == "__main__":
    fn = './scene_complex_v1.json'
    change_unsafe(fn)