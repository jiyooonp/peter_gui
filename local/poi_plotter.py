import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np
from sklearn.mixture import GaussianMixture
import mpl_toolkits.mplot3d.art3d as art3d

peduncles= np.genfromtxt(fname="assets/peduncles_10242023.csv", delimiter=",")
peppers= np.genfromtxt(fname="assets/peppers_10242023.csv", delimiter=",")

min_peppers, max_peppers = min(peppers[:, -1]), max(peppers[:, -1])
min_peduncles, max_peduncles = min(peduncles[:, -1]), max(peduncles[:, -1])

peduncles = peduncles[:, :-1].T
peppers = peppers[:, :-1].T

# import pdb; pdb.set_trace()
peduncles = peduncles[:, (peduncles[0] > 0.3)]


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_ellipsoid_3d(ax, center, cov_matrix, nstd=1., color='blue', alpha=0.3):
    """Plot 3D ellipsoid on a given ax."""
    # Find the eigenvalues and eigenvectors
    eigvals, eigvecs = np.linalg.eigh(cov_matrix)
    
    # Define a rotation based on eigenvectors
    rotation = np.dot(eigvecs, np.diag(np.sqrt(eigvals)))

    # Generate samples on the unit sphere
    phi = np.linspace(0, 2 * np.pi, 50)
    theta = np.linspace(0, np.pi, 50)
    PHI, THETA = np.meshgrid(phi, theta)
    X = np.sin(THETA) * np.cos(PHI)
    Y = np.sin(THETA) * np.sin(PHI)
    Z = np.cos(THETA)
    
    # Apply the rotations and scaling
    data = np.array([X.flatten(), Y.flatten(), Z.flatten()])
    rotated_data = np.dot(rotation, data * nstd)
    
    # Add the center offset and reshape
    rotated_data = rotated_data + center[:, np.newaxis]
    rotated_data = rotated_data.reshape(3, X.shape[0], X.shape[1])
    
    # Plot
    ax.plot_surface(rotated_data[0], rotated_data[1], rotated_data[2],
                    rstride=2, cstride=2, color=color, alpha=alpha, linewidth=0)
    # ax.scatter(*center, color='red', s=100, label='Center')

min_peduncle_clusters = GaussianMixture(n_components=int(min_peduncles))
max_peduncle_clusters = GaussianMixture(n_components=int(max_peduncles))

min_peduncle_clusters.fit(peduncles.T)
max_peduncle_clusters.fit(peduncles.T)

fig1 = plt.figure()
ax1 = fig1.add_subplot(projection='3d')
ax1.scatter(*peduncles)

lim = max(np.max(peduncles, axis=1))

for exp, cov in zip(min_peduncle_clusters.means_, min_peduncle_clusters.covariances_):
    plot_ellipsoid_3d(ax1, exp, cov, nstd=3, color='red', alpha=0.1)
    plot_ellipsoid_3d(ax1, exp, cov, nstd=2, color='grey',alpha=0.2)
    plot_ellipsoid_3d(ax1, exp, cov, nstd=1, color='green', alpha=0.3)

ax1.set_title(f"Minimum Peduncles Detected {min_peduncles}")
ax1.set_xlabel("x-axis (m)")
ax1.set_ylabel("y-axis (m)")
ax1.set_zlabel("z-axis (m)")
ax1.set_xlim(0, lim)
ax1.set_ylim(0, lim)
ax1.set_zlim(0, lim)
ax1.set_aspect("equal")


fig2 = plt.figure()
ax2 = fig2.add_subplot(projection='3d')
ax2.scatter(*peduncles)

ax2.set_title(f"Maximum Peduncles Detected {max_peduncles}")
ax2.set_xlabel("x-axis (m)")
ax2.set_ylabel("y-axis (m)")
ax2.set_zlabel("z-axis (m)")
ax2.set_xlim(0, lim)
ax2.set_ylim(0, lim)
ax2.set_zlim(0, lim)
ax2.set_aspect("equal")

for exp, cov in zip(max_peduncle_clusters.means_, max_peduncle_clusters.covariances_):
    plot_ellipsoid_3d(ax2, exp, cov, nstd=3, color='red', alpha=0.1)
    plot_ellipsoid_3d(ax2, exp, cov, nstd=2, color='grey', alpha=0.2)
    plot_ellipsoid_3d(ax2, exp, cov, nstd=1, color='green', alpha=0.3)



# cbar = plt.colorbar(scatter, ax=ax2)
# cbar.set_label('Depth (m)')

# ax3 = fig.add_subplot(133)
# ax3.set_xlim(-lim, lim)
# ax3.set_ylim(-lim, lim)
# ax3.set_xlabel("Distance (m)")
# ax3.set_ylabel("Distance (m)")
# ax3.set_aspect("equal")

# # Add multiple ellipses for different sigma levels
# for sigma in [1, 2, 3]:
#     width, height = 2 * np.sqrt(eigenvalues_xz * sigma)  # 2*sqrt(eigenvalue) gives a 1-sigma ellipse
#     angle = np.degrees(np.arctan2(*eigenvectors_xz[:, 0][::-1]))
#     ellipse = Ellipse(xy=(0, 0), width=width, height=height, angle=angle, edgecolor='r', facecolor='none')
#     ax3.add_patch(ellipse)
    
# scatter = ax3.scatter(poi_rs[0], poi_rs[-1], c=poi_rs[-1], cmap='viridis')

# # cbar2 = plt.colorbar(scatter, ax=ax3)
# # cbar2.set_label('Depth (m)')

# peppers = GaussianMixture(n_components=1)
# peppers.fit(poi_rs[:2].T)

# eigenvalues_xy, eigenvectors_xy = np.linalg.eig(peppers.covariances_[0])

# # Add multiple ellipses for different sigma levels
# for sigma in [1, 2, 3]:
#     width, height = 2 * np.sqrt(eigenvalues_xy * sigma)  # 2*sqrt(eigenvalue) gives a 1-sigma ellipse
#     angle = np.degrees(np.arctan2(*eigenvectors_xy[:, 0][::-1]))
#     ellipse = Ellipse(xy=(0, 0), width=width, height=height, angle=angle, edgecolor='b', facecolor='none')
#     ax2.add_patch(ellipse)
    
# peppers = GaussianMixture(n_components=1)
# print(np.hstack((poi_rs[:, 0], poi_rs[:, 2])).shape, poi_rs.shape)
# peppers.fit(np.vstack((poi_rs[0], poi_rs[2])).T)


# eigenvalues_xz, eigenvectors_xz = np.linalg.eig(peppers.covariances_[0])

# # Add multiple ellipses for different sigma levels
# for sigma in [1, 2, 3]:
#     width, height = 2 * np.sqrt(eigenvalues_xz * sigma)  # 2*sqrt(eigenvalue) gives a 1-sigma ellipse
#     angle = np.degrees(np.arctan2(*eigenvectors_xz[:, 0][::-1]))
#     ellipse = Ellipse(xy=(0, 0), width=width, height=height, angle=angle, edgecolor='b', facecolor='none')
#     ax3.add_patch(ellipse)
    
    
# fig2 = plt.figure()
# ax4 = fig2.add_subplot()
    
# fake_mean = np.mean(poi_rs[:2]) - np.array([1, 1])
# fake_data = np.random.multivariate_normal(mean=fake_mean, cov=np.array([[0.003201683, 0.09738461], [0.01738461, 0.005846042]]), size=36)
# print(np.cov(poi_rs[:2]))
# print((fake_mean.shape, np.cov(poi_rs[:2]).shape, fake_data.T.shape, poi_rs[:2].shape))

# new_data = np.hstack((fake_data.T, poi_rs[:2])).T

# peppers = GaussianMixture(n_components=2, n_init=3)
# peppers.fit(new_data)

# for i in range(len(peppers.covariances_)):

#     eigenvalues_xz, eigenvectors_xz = np.linalg.eig(peppers.covariances_[i])

#     # Add multiple ellipses for different sigma levels
#     for sigma in [1, 2, 3]:
#         width, height = 2 * np.sqrt(eigenvalues_xz * sigma)  # 2*sqrt(eigenvalue) gives a 1-sigma ellipse
#         angle = np.degrees(np.arctan2(*eigenvectors_xz[:, 0][::-1]))
#         ellipse = Ellipse(xy=peppers.means_[i], width=width, height=height, angle=angle, edgecolor='b', facecolor='none')
#         ax4.add_patch(ellipse)

# ax4.scatter(*new_data.T)
# ax4.set_xlim(np.min(new_data), np.max(new_data))
# ax4.set_ylim(np.min(new_data), np.max(new_data))
# ax4.set_xlabel("Meters")
# ax4.set_ylabel("Meters")
# ax4.set_aspect("equal")

    

    
plt.show()