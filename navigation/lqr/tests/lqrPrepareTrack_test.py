#!/usr/bin/env python3
# pylint: disable=all
# mypy: ignore-errors
import unittest
import numpy as np
import scipy.interpolate as interpolate
from lqr import sideofLine, distToP, interpTrack, calcSplines, Track, splineApprox


class test_lqrPrepareTrack(unittest.TestCase):
    def test_sideofLine(self):
        lineStart = np.array([0, 0])
        lineEnd = np.array([1, 1])
        pointZ = np.array([0, 1])
        side = sideofLine(lineStart, lineEnd, pointZ)
        self.assertEqual(side, 1)

    def test_disttoP(self):
        t_glob = np.linspace(0, 1, 4)  # Example time values
        y = np.array([0, 0, 0, 0])  # Example path values
        w = np.ones(4)  # Example weights for path values
        path = interpolate.splrep(t_glob, y, w)  # Example path spline
        p = np.ones(4)  # Example point p
        distance = distToP(t_glob, path, p)
        self.assertEqual(distance, 2.0)

    def test_CalcSplines(self):
        path = np.array([[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]])
        xCoeffs, yCoeffs, splineAlpha, normVecNormal, noSplines = calcSplines(path)

        self.assertTrue(
            np.allclose(
                xCoeffs,
                np.array(
                    [
                        [0.0, 1.33333333, -0.25, -0.08333333],
                        [1.0, 0.58333333, -0.5, -0.08333333],
                        [1.0, -0.66666667, -0.75, 0.41666667],
                        [0.0, -0.91666667, 0.5, 0.41666667],
                    ]
                ),
            )
        )
        self.assertTrue(
            np.allclose(
                yCoeffs,
                np.array(
                    [
                        [0.0, -1.33333333, 1.75, -0.41666667],
                        [0.0, 0.91666667, 0.5, -0.41666667],
                        [1.0, 0.66666667, -0.75, 0.08333333],
                        [1.0, -0.58333333, -0.5, 0.08333333],
                    ]
                ),
            )
        )
        self.assertTrue(
            np.allclose(
                splineAlpha,
                np.array(
                    [
                        [
                            1.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        ],
                        [
                            1.0,
                            1.0,
                            1.0,
                            1.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        ],
                        [
                            0.0,
                            1.0,
                            2.0,
                            3.0,
                            0.0,
                            -1.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        ],
                        [
                            0.0,
                            0.0,
                            2.0,
                            6.0,
                            0.0,
                            0.0,
                            -2.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        ],
                        [
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        ],
                        [
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                            1.0,
                            1.0,
                            1.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        ],
                        [
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                            2.0,
                            3.0,
                            0.0,
                            -1.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        ],
                        [
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            2.0,
                            6.0,
                            0.0,
                            0.0,
                            -2.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        ],
                        [
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        ],
                        [
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                            1.0,
                            1.0,
                            1.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        ],
                        [
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                            2.0,
                            3.0,
                            0.0,
                            -1.0,
                            0.0,
                            0.0,
                        ],
                        [
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            2.0,
                            6.0,
                            0.0,
                            0.0,
                            -2.0,
                            0.0,
                        ],
                        [
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                            0.0,
                            0.0,
                            0.0,
                        ],
                        [
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                            1.0,
                            1.0,
                            1.0,
                        ],
                        [
                            0.0,
                            1.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            -1.0,
                            -2.0,
                            -3.0,
                        ],
                        [
                            0.0,
                            0.0,
                            2.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            -2.0,
                            -6.0,
                        ],
                    ]
                ),
            )
        )
        self.assertTrue(
            np.allclose(
                normVecNormal,
                np.array(
                    [
                        [-0.70710678, -0.70710678],
                        [0.84366149, -0.53687549],
                        [0.70710678, 0.70710678],
                        [-0.53687549, 0.84366149],
                    ]
                ),
            )
        )
        self.assertEqual(
            noSplines,
            4,
        )

    def test_trackInterp(self):
        track = np.array(
            [
                [0.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [2.0, 0.0, 0.0, 0.0],
                [3.0, 0.0, 0.0, 0.0],
                [4.0, 0.0, 0.0, 0.0],
                [5.0, 0.0, 0.0, 0.0],
                [6.0, 0.0, 0.0, 0.0],
                [7.0, 0.0, 0.0, 0.0],
                [8.0, 0.0, 0.0, 0.0],
                [9.0, 0.0, 0.0, 0.0],
                [10.0, 0.0, 0.0, 0.0],
            ]
        )
        trackInterp = interpTrack(track, 0.5)
        self.assertTrue(
            np.allclose(
                trackInterp,
                np.array(
                    [
                        [0.0, 0.0, 0.0, 0.0],
                        [0.5, 0.0, 0.0, 0.0],
                        [1.0, 0.0, 0.0, 0.0],
                        [1.5, 0.0, 0.0, 0.0],
                        [2.0, 0.0, 0.0, 0.0],
                        [2.5, 0.0, 0.0, 0.0],
                        [3.0, 0.0, 0.0, 0.0],
                        [3.5, 0.0, 0.0, 0.0],
                        [4.0, 0.0, 0.0, 0.0],
                        [4.5, 0.0, 0.0, 0.0],
                        [5.0, 0.0, 0.0, 0.0],
                        [5.5, 0.0, 0.0, 0.0],
                        [6.0, 0.0, 0.0, 0.0],
                        [6.5, 0.0, 0.0, 0.0],
                        [7.0, 0.0, 0.0, 0.0],
                        [7.5, 0.0, 0.0, 0.0],
                        [8.0, 0.0, 0.0, 0.0],
                        [8.5, 0.0, 0.0, 0.0],
                        [9.0, 0.0, 0.0, 0.0],
                        [9.5, 0.0, 0.0, 0.0],
                    ]
                ),
            )
        )

    def test_SplineApprox(self):
        # make a circular track with 5m radius
        refTrack = np.zeros((10, 4))
        refTrack[:, 0] = 5 * np.cos(np.linspace(0, 2 * np.pi, 10))
        refTrack[:, 1] = 5 * np.sin(np.linspace(0, 2 * np.pi, 10))
        refTrack[:, 2] = 0.5
        refTrack[:, 3] = 0.5

        # Load track
        track = Track(refTrack=refTrack)

        # Test splineApprox
        trackSmooth = splineApprox(track=track.original)
        self.assertTrue(
            np.allclose(
                trackSmooth,
                np.array(
                    [
                        [4.88130958e00, 5.93193725e-05, 6.18690434e-01, 3.81309566e-01],
                        [3.78188002e00, 2.45329784e00, 1.01097669e00, -1.09766854e-02],
                        [1.25115954e00, 4.09188991e00, 1.17510771e00, -1.75107708e-01],
                        [-1.55401601e00, 4.14385099e00, 1.09368696e00, -9.36869601e-02],
                        [-3.69320923e00, 2.53002759e00, 1.02011291e00, -2.01129123e-02],
                        [-4.49604346e00, 3.31408604e-02, 1.03427512e00, -3.42751215e-02],
                        [-3.62325461e00, -2.49441894e00, 1.07542423e00, -7.54242324e-02],
                        [-1.51369475e00, -4.16124672e00, 1.10234784e00, -1.02347842e-01],
                        [1.25490697e00, -4.12733373e00, 1.14285051e00, -1.42850510e-01],
                        [3.77654485e00, -2.46915253e00, 1.00264714e00, -2.64714170e-03],
                    ]
                ),
            )
        )


if __name__ == "__main__":
    unittest.main()
