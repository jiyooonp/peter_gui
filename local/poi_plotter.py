import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np
from sklearn.mixture import GaussianMixture

poi= np.genfromtxt(fname="assets/gpd_poi.csv", delimiter=",")
poi_rs= np.genfromtxt(fname="assets/gpd_poi_rs.csv", delimiter=",")

poi = poi.T
poi_rs = poi_rs.T

fig = plt.figure()

# Compute the covariance matrix
cov_matrix_xy = np.cov(poi_rs[:2])
cov_matrix_xz = np.cov(poi_rs[0], poi_rs[-1])

# Eigen decomposition
eigenvalues_xy, eigenvectors_xy = np.linalg.eig(cov_matrix_xy)
eigenvalues_xz, eigenvectors_xz = np.linalg.eig(cov_matrix_xz)


ax1 = fig.add_subplot(131, projection='3d')
ax1.scatter(*poi)
ax1.set_xlim(np.min(poi), np.max(poi))
ax1.set_ylim(np.min(poi), np.max(poi))
ax1.set_zlim(np.min(poi), np.max(poi))
ax1.set_xlabel("Meters")
ax1.set_ylabel("Meters")
ax1.set_zlabel("Meters")
ax1.set_aspect("equal")

lim = max(abs(np.min(poi_rs[:2])), abs(np.max(poi_rs[:2])))

ax2 = fig.add_subplot(132)
ax2.set_xlim(-lim, lim)
ax2.set_ylim(-lim, lim)
ax2.set_xlabel("Distance (m)")
ax2.set_ylabel("Distance (m)")
ax2.set_aspect("equal")

# Add multiple ellipses for different sigma levels
for sigma in [1, 2, 3]:
    width, height = 2 * np.sqrt(eigenvalues_xy * sigma)  # 2*sqrt(eigenvalue) gives a 1-sigma ellipse
    angle = np.degrees(np.arctan2(*eigenvectors_xy[:, 0][::-1]))
    ellipse = Ellipse(xy=(0, 0), width=width, height=height, angle=angle, edgecolor='r', facecolor='none')
    ax2.add_patch(ellipse)
    
scatter = ax2.scatter(*poi_rs[:2], c=poi_rs[-1], cmap='viridis')

# cbar = plt.colorbar(scatter, ax=ax2)
# cbar.set_label('Depth (m)')

ax3 = fig.add_subplot(133)
ax3.set_xlim(-lim, lim)
ax3.set_ylim(-lim, lim)
ax3.set_xlabel("Distance (m)")
ax3.set_ylabel("Distance (m)")
ax3.set_aspect("equal")

# Add multiple ellipses for different sigma levels
for sigma in [1, 2, 3]:
    width, height = 2 * np.sqrt(eigenvalues_xz * sigma)  # 2*sqrt(eigenvalue) gives a 1-sigma ellipse
    angle = np.degrees(np.arctan2(*eigenvectors_xz[:, 0][::-1]))
    ellipse = Ellipse(xy=(0, 0), width=width, height=height, angle=angle, edgecolor='r', facecolor='none')
    ax3.add_patch(ellipse)
    
scatter = ax3.scatter(poi_rs[0], poi_rs[-1], c=poi_rs[-1], cmap='viridis')

# cbar2 = plt.colorbar(scatter, ax=ax3)
# cbar2.set_label('Depth (m)')

peppers = GaussianMixture(n_components=1)
peppers.fit(poi_rs[:2].T)

eigenvalues_xy, eigenvectors_xy = np.linalg.eig(peppers.covariances_[0])

# Add multiple ellipses for different sigma levels
for sigma in [1, 2, 3]:
    width, height = 2 * np.sqrt(eigenvalues_xy * sigma)  # 2*sqrt(eigenvalue) gives a 1-sigma ellipse
    angle = np.degrees(np.arctan2(*eigenvectors_xy[:, 0][::-1]))
    ellipse = Ellipse(xy=(0, 0), width=width, height=height, angle=angle, edgecolor='b', facecolor='none')
    ax2.add_patch(ellipse)
    
peppers = GaussianMixture(n_components=1)
print(np.hstack((poi_rs[:, 0], poi_rs[:, 2])).shape, poi_rs.shape)
peppers.fit(np.vstack((poi_rs[0], poi_rs[2])).T)


eigenvalues_xz, eigenvectors_xz = np.linalg.eig(peppers.covariances_[0])

# Add multiple ellipses for different sigma levels
for sigma in [1, 2, 3]:
    width, height = 2 * np.sqrt(eigenvalues_xz * sigma)  # 2*sqrt(eigenvalue) gives a 1-sigma ellipse
    angle = np.degrees(np.arctan2(*eigenvectors_xz[:, 0][::-1]))
    ellipse = Ellipse(xy=(0, 0), width=width, height=height, angle=angle, edgecolor='b', facecolor='none')
    ax3.add_patch(ellipse)
    
    
fig2 = plt.figure()
ax4 = fig2.add_subplot()
    
fake_mean = np.mean(poi_rs[:2]) - np.array([1, 1])
fake_data = np.random.multivariate_normal(mean=fake_mean, cov=np.array([[0.003201683, 0.09738461], [0.01738461, 0.005846042]]), size=36)
print(np.cov(poi_rs[:2]))
print((fake_mean.shape, np.cov(poi_rs[:2]).shape, fake_data.T.shape, poi_rs[:2].shape))

new_data = np.hstack((fake_data.T, poi_rs[:2])).T

peppers = GaussianMixture(n_components=2, n_init=3)
peppers.fit(new_data)

for i in range(len(peppers.covariances_)):

    eigenvalues_xz, eigenvectors_xz = np.linalg.eig(peppers.covariances_[i])

    # Add multiple ellipses for different sigma levels
    for sigma in [1, 2, 3]:
        width, height = 2 * np.sqrt(eigenvalues_xz * sigma)  # 2*sqrt(eigenvalue) gives a 1-sigma ellipse
        angle = np.degrees(np.arctan2(*eigenvectors_xz[:, 0][::-1]))
        ellipse = Ellipse(xy=peppers.means_[i], width=width, height=height, angle=angle, edgecolor='b', facecolor='none')
        ax4.add_patch(ellipse)

ax4.scatter(*new_data.T)
ax4.set_xlim(np.min(new_data), np.max(new_data))
ax4.set_ylim(np.min(new_data), np.max(new_data))
ax4.set_xlabel("Meters")
ax4.set_ylabel("Meters")
ax4.set_aspect("equal")

    

    
plt.show()