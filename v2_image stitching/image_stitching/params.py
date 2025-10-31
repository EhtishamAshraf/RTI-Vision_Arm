# data i/o params
DATASET_PATH = r'C:\Users\ad\Downloads\Thesis\data\retrato de lola flores\dome'
DATASET_DOWNSAMPLE_RATIO = 0.1

#acq params
COMPUTE_AND_SAVE_NORMALS = False
PLOT_AND_SAVE_LIGHT_POSITIONS = False

# Composites computation
OVERWRITE_EXISTING_COMPOSITES = False
COMPOSITE_IMAGE_SUFFIX = "_composite_exaggerated" # "_original" or "_composite_exaggerated" or "_composite"

# Compute normals params
PS_METHOD = "L2_SOLVER"    # Least-squares
# PS_METHOD = "L1_SOLVER_MULTICORE"    # L1 residual minimization
# PS_METHOD = "SBL_SOLVER_MULTICORE"    # Sparse Bayesian Learning
# PS_METHOD = "RPCA_SOLVER"    # Robust PCA

# RTI
RTI_NET_MAX_NUMBER_OF_IMAGES = 45
RTI_NET_EPOCHS = 100
RTI_NET_SAVE_MODEL_EVERY_N_EPOCHS = 10
RTI_NET_PATCH_SIZE = 128
RTI_NET_PATCHES_PER_IMAGE = 4


#Feature extraction params
MINIMUM_MATCH_POINTS = 10 # default 20
CONFIDENCE_THRESH = 0.05 # default 65
NUM_KEYPOINTS = 1000 # default 1000
THRESHOLD = 0.8 # default 0.8

# light correction params
LIGHT_CORRECTION_METHOD = 'homomorphic' # 'lam_inv_sq' or 'lab'	or 'ssr' or 'homomorphic' or 'nn'
DISTANCE_CORRECTION_GAIN = 1.2 #default 1.2
LAMBERT_CORRECTION_GAIN = 1.0 #default 1.0
SURFACE_PHYSCIAL_SIZE = [0.20, 0.60] #default [0.250, 0.120]


#Homomorphic filter params
GAMMA_HIGH = 1.5
GAMMA_LOW = 0.5
D0 = 30
C = 2

# Retinex params
RETINEX_SIGMA = 400 # default 400

#stitching, registration params
OVERLAP_PERCENTAGE = 10 #default 10
SAVE_OVERLAP_AS_CSV = False #save overlapping indices and pixel values as csv

#blending params
BLENDING_ALGO = 'poisson' # 'alpha' or 'poisson' or 'multiband' or ''feather' or 'pyramid' 
BLENDING_ALPHA = 0.5 #default 0.5