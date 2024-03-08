"""
Description: Algorithm that for sorted cone configurations, find the number of cones
that are on the wrong side of the track. For example, of we are considering the left
edge of the track, we do not want to see any cones to the right of them.
"""
from collections import deque
from sys import maxsize
from typing import Dict, Optional, Tuple

import numpy as np

from src.cone_matching.match_directions import calculateSearchDirectionForOne
from src.types_file.types import BoolArray, FloatArray, IntArray
from src.utils.cone_types import ConeTypes
from src.utils.math_utils import (
    myCdistSqEuclidean,
    myNjit,
    vecAngleBetween,
)

SEARCH_DIRECTIONS_CACHE_KEY_TYPE = Tuple[int, int, int]
SEARCH_DIRECTIONS_CACHE_TYPE = Dict[SEARCH_DIRECTIONS_CACHE_KEY_TYPE, FloatArray]

SENTINEL_VALUE = maxsize - 10

ANGLE_MASK_CACHE_KEY_TYPE = Tuple[Tuple[int, int, int], int, int]
ANGLE_MASK_CACHE_TYPE = Dict[ANGLE_MASK_CACHE_KEY_TYPE, Tuple[bool, bool]]


class NearbyConeSearcher:
    def __init__(self) -> None:
        self.cachesCache: deque[
            tuple[tuple[int, ConeTypes], tuple[dict, dict, FloatArray, FloatArray]]
        ] = deque(maxlen=20)

    def getCaches(
        self, cones: np.ndarray, coneType: ConeTypes
    ) -> tuple[dict, dict, FloatArray, FloatArray]:
        arrayBuffer = cones.tobytes()
        arrayHash = hash(arrayBuffer)
        cacheKey = (arrayHash, coneType)

        try:
            indexOfHashedValues = next(
                i for i, (k, _) in enumerate(self.cachesCache) if k == cacheKey
            )
        except StopIteration:
            indexOfHashedValues = None
        if indexOfHashedValues is None:
            conesXY = cones[:, :2]
            distanceMatrixSquare = myCdistSqEuclidean(conesXY, conesXY)
            np.fill_diagonal(distanceMatrixSquare, 1e7)
            conesToCones = conesXY - conesXY[:, None]

            self.cachesCache.append(
                (
                    cacheKey,
                    (
                        createSearchDirectionsCache(),
                        createAngleCache(),
                        distanceMatrixSquare,
                        conesToCones,
                    ),
                )
            )
            indexOfHashedValues = -1
        return self.cachesCache[indexOfHashedValues][1]

    def numberOfConesOnEachSideForEachConfig(
        self,
        cones: np.ndarray,
        configs: np.ndarray,
        coneType: ConeTypes,
        maxDistance: float,
        maxAngle: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        cachedValues = self.getCaches(cones, coneType)
        return _implNumberOfConesOnEachSideForEachConfig(
            cones, configs, coneType, maxDistance, maxAngle, *cachedValues[:]
        )


NEARBY_CONE_SEARCH = NearbyConeSearcher()


def _implNumberOfConesOnEachSideForEachConfig(
    cones: FloatArray,
    configs: IntArray,
    coneType: ConeTypes,
    searchDistance: float,
    searchAngle: float,
    existingSearchDirectionsCache: Optional[SEARCH_DIRECTIONS_CACHE_TYPE] = None,
    existingAngleMaskCache: Optional[ANGLE_MASK_CACHE_TYPE] = None,
    distanceMatrixSquare: Optional[FloatArray] = None,
    conesToConesVecs: Optional[FloatArray] = None,
) -> tuple[IntArray, IntArray]:
    """
    For each configuration, find the number od cones that are on the expected side of
    the track, and the number of cones that are on the wrong side of the track.

    Args:
        cones: array of cone positions and types
        configs: array of sorted cone configurations
        coneType: type of cone to consider
        searchDistance: the distance to search for cones
        searchAngle: the angle to search for cones

    Returns:
        A tuple of two arrays, the first is the number of cones on the correct side of
        the track, the second is the number of cones on the wrong side of the track
    """
    conesXY = cones[:, :2]

    idxsInAllConfigs = np.unique(configs)
    idxsInAllConfigs = idxsInAllConfigs[idxsInAllConfigs != -1]

    if conesToConesVecs is None:
        conesToConesVecs = conesXY - np.expand_dims(conesXY, 1)

    if distanceMatrixSquare is None:
        distanceMatrixSquare = myCdistSqEuclidean(conesXY, conesXY)
        np.fill_diagonal(distanceMatrixSquare, 1e6)

    distanceMatrixMask = distanceMatrixSquare < searchDistance * searchDistance

    searchDirectionsCache = preCalculateSearchDirections(
        cones, configs, coneType, existingSearchDirectionsCache
    )

    closeIdxs = findNearbyConesForIdxs(idxsInAllConfigs, distanceMatrixSquare, searchDistance)

    nBadConesForAll = np.zeros(len(configs), dtype=np.int_)
    nGoodConesForAll = np.zeros(len(configs), dtype=np.int_)

    if existingAngleMaskCache is None:
        angleCache = createAngleCache()
    else:
        angleCache = existingAngleMaskCache

    for i, c in enumerate(configs):
        c = c[c != -1]

        extraIdxs = sortedSetDiff(idxsInAllConfigs, c)
        otherIdxs = np.concatenate((closeIdxs, extraIdxs))

        for j in range(len(c)):
            if j == 0:
                key = (c[j], SENTINEL_VALUE, c[j + 1])
            elif j == len(c) - 1:
                key = (c[j - 1], SENTINEL_VALUE, c[j])
            else:
                key = (c[j - 1], c[j], c[j + 1])

            maskGood, maskBad = calculateVisibleConesForOneCone(
                c[j],
                distanceMatrixMask,
                key,
                conesToConesVecs,
                searchAngle,
                searchDirectionsCache,
                angleCache,
                idxsToCheck=otherIdxs,
            )

            nGoodConesForAll[i] += maskGood.sum()
            nBadConesForAll[i] += maskBad.sum()

    return nGoodConesForAll, nBadConesForAll


def preCalculateSearchDirections(
    cones: FloatArray,
    configs: IntArray,
    coneType: ConeTypes,
    existingCache: Optional[SEARCH_DIRECTIONS_CACHE_TYPE] = None,
) -> SEARCH_DIRECTIONS_CACHE_TYPE:
    conesXY = cones[:, :2]
    if existingCache is not None:
        cache = existingCache
    else:
        cache = createSearchDirectionsCache()

    for c in configs:
        c = c[c != -1]
        assert len(c) >= 2

        keyFirst = (c[0], SENTINEL_VALUE, c[1])
        keyLast = (c[-2], SENTINEL_VALUE, c[-1])

        calculateMatchSearchDirectionForOneIfNotInCache(conesXY, keyFirst, coneType, cache)
        calculateMatchSearchDirectionForOneIfNotInCache(conesXY, keyLast, coneType, cache)

        for j in range(1, len(c) - 1):
            key = (c[j - 1], c[j], c[j + 1])
            calculateMatchSearchDirectionForOneIfNotInCache(conesXY, key, coneType, cache)

    return cache


def calculateMatchSearchDirectionForOneIfNotInCache(
    conesXY: FloatArray,
    key: SEARCH_DIRECTIONS_CACHE_KEY_TYPE,
    coneType: ConeTypes,
    cacheDict: SEARCH_DIRECTIONS_CACHE_TYPE,
) -> FloatArray:
    if key not in cacheDict:
        cacheDict[key] = calculateSearchDirectionForOne(conesXY, key[0::2], coneType)

    return cacheDict[key]


def findNearbyConesForIdxs(
    idxs: IntArray, distanceMatrixSquare: FloatArray, searchRange: float
) -> IntArray:
    mask = distanceMatrixSquare[idxs] < searchRange * searchRange
    allIdxs = np.unique(mask.nonzero()[1])

    # calculate the set deffrenece between allIdxs and idxs
    return sortedSetDiff(allIdxs, idxs)


def sortedSetDiff(a: IntArray, b: IntArray) -> IntArray:
    """Returns the set diffrence between a and b, assume a,b are sorted"""
    # we cannot use np.setdiff1d because it is not supported in numba
    mask = np.ones(len(a), dtype=np.bool_)
    indices = np.clip(np.searchsorted(a, b), a_min=0, a_max=len(a) - 1)
    mask[indices] = False
    return a[mask]


def calculateVisibleConesForOneCone(
    coneIdx: int,
    coneWithinDistanceMatrixMask: BoolArray,
    searchDirectionKey: SEARCH_DIRECTIONS_CACHE_KEY_TYPE,
    conesToConesVecs: FloatArray,
    searchAngle: float,
    searchDirectionCache: SEARCH_DIRECTIONS_CACHE_TYPE,
    anglesBetweenSearchDirectionAndOtherConeCache: ANGLE_MASK_CACHE_TYPE,
    idxsToCheck,
) -> tuple[BoolArray, BoolArray]:
    angleGoodMask = np.zeros(len(idxsToCheck), dtype=np.bool_)
    angleBadMask = np.zeros(len(idxsToCheck), dtype=np.bool_)
    for i in range(len(idxsToCheck)):
        idx = idxsToCheck[i]

        if not coneWithinDistanceMatrixMask[coneIdx, idx]:
            continue

        (
            valueGood,
            valueBad,
        ) = angleBetweenSearchDirectionOfConeAndOtherConeIsTooLargeIfNotInCache(
            conesToConesVecs,
            searchDirectionKey,
            coneIdx,
            idx,
            searchDirectionCache,
            anglesBetweenSearchDirectionAndOtherConeCache,
            searchAngle,
        )
        angleGoodMask[i] = valueGood
        angleBadMask[i] = valueBad

    maskDistance = coneWithinDistanceMatrixMask[coneIdx]

    goodMask = angleGoodMask & maskDistance[idxsToCheck]
    badMask = angleBadMask & maskDistance[idxsToCheck]

    return goodMask, badMask


def angleBetweenSearchDirectionOfConeAndOtherConeIsTooLargeIfNotInCache(
    allConeDirections: FloatArray,
    directionsKey: SEARCH_DIRECTIONS_CACHE_KEY_TYPE,
    coneIdx: int,
    otherConeIdx: int,
    searchDirectionCache: SEARCH_DIRECTIONS_CACHE_TYPE,
    angleCache: ANGLE_MASK_CACHE_TYPE,
    searchAngle: float,
) -> bool:
    key = (directionsKey, coneIdx, otherConeIdx)
    if key not in angleCache:
        angleCache[key] = angleBetweenSearchDirectionOfConeAndOtherConeIsTooLarge(
            allConeDirections,
            directionsKey,
            coneIdx,
            otherConeIdx,
            searchDirectionCache,
            searchAngle,
        )

    return angleCache[key]


def angleBetweenSearchDirectionOfConeAndOtherConeIsTooLarge(
    allConeDirections: FloatArray,
    directionsKey: SEARCH_DIRECTIONS_CACHE_KEY_TYPE,
    coneIdx: int,
    otherConeIdx: int,
    searchDirectionCache: SEARCH_DIRECTIONS_CACHE_TYPE,
    searchAngle: float,
) -> tuple[bool, bool]:
    fromConeToOtherCone = allConeDirections[coneIdx, otherConeIdx]

    searchDirection = searchDirectionCache[directionsKey]

    goodAngle = vecAngleBetween(fromConeToOtherCone, searchDirection) < searchAngle / 2
    badAngle = vecAngleBetween(fromConeToOtherCone, -searchDirection) < searchAngle / 2

    return goodAngle, badAngle


def numberConesOnEachSideForEachConfig(
    cones: np.ndarray,
    configs: np.ndarray,
    coneType: ConeTypes,
    maxDistance: float,
    maxAngle: float,
) -> tuple[np.ndarray, np.ndarray]:
    return NEARBY_CONE_SEARCH.numberOfConesOnEachSideForEachConfig(
        cones, configs, coneType, maxDistance, maxAngle
    )


def createSearchDirectionsCache() -> SEARCH_DIRECTIONS_CACHE_TYPE:
    return {(SENTINEL_VALUE, SENTINEL_VALUE, SENTINEL_VALUE): np.array([-1.0, -1.0])}


def createAngleCache() -> ANGLE_MASK_CACHE_TYPE:
    # the key is a tuple which consists of the following
    # - the key of the search direction cache (previous cone, current cone, next cone)
    # - the index of the cone we are considiring
    # - the index of the cone we are comparing to
    d = {
        (
            (SENTINEL_VALUE, SENTINEL_VALUE, SENTINEL_VALUE),
            SENTINEL_VALUE,
            SENTINEL_VALUE,
        ): (False, False),
    }

    return d
