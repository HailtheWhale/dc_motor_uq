from concurrent.futures import ProcessPoolExecutor

def f(x):
    return x*x

with ProcessPoolExecutor() as executor:
    print(list(executor.map(f, [1,2,3])))