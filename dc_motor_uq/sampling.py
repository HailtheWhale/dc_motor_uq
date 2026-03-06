import numpy as np
from scipy.stats import qmc, norm

def monte_carlo_sampling(params_mean:np.ndarray,
                         params_std_dev:np.ndarray,
                        n_samples:int=1000):
    
# Make matrix to store vals. n_samples x n_params
    params = np.zeros((n_samples, len(params_mean)))
# All following normal distributions. 
    for i,val in enumerate(params_mean):
        params[:,i] = np.random.normal(val, params_std_dev[i], n_samples)
    return params

def lhs_sampling(params_mean:np.ndarray,
                 params_std_dev:np.ndarray,
                 n_samples:int=1000):
    
    means = np.array(params_mean)
    std = np.array(params_std_dev)

    sampler = qmc.LatinHypercube(d=len(params_mean))
    unit_samples = sampler.random(n=n_samples) # shape: (n_samples, n_params)

    params = norm.ppf(unit_samples, loc = means, scale = std)
    return params
    
def uniform_sampling(params_mean:np.ndarray,
                    params_std_dev:np.ndarray,
                    n_samples:int=1000):
    
# Make matrix to store vals. n_samples x n_params
    params=np.zeros((n_samples,len(params_mean)))

    for i,val in enumerate(params_mean):
    # Based on the 3 sigma rule where 99.7% of values fall within 3 std devs
    # from the mean, want to sample for values within that. 
        bnd_low  = val - 3 * params_std_dev[i]
        bnd_high = val + 3 * params_std_dev[i]
        params[:,i] = np.random.uniform(low = bnd_low, high = bnd_high, size = n_samples)
    return params
