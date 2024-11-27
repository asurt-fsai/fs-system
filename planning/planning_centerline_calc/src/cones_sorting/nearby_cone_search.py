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
    vecAngleBetween,
)

SearchDirectionsCacheKeyType = Tuple[int, int, int]
SearchDirectionsCacheType = Dict[SearchDirectionsCacheKeyType, FloatArray]

SENTINEL_VALUE = maxsize - 10

AngleMaskCacheKeyType = Tuple[Tuple[int, int, int], int, int]
AngleMaskCacheType = Dict[AngleMaskCacheKeyType, Tuple[bool, bool]]


class NearbyConeSearcher:
    """
    A class that performs nearby cone search operations.

    Methods:
        getCaches(cones: np.ndarray, coneType: ConeTypes)
        -> tuple[dict, dict, FloatArray, FloatArray]:
            Retrieves the cached values for the given cones and cone type.

        numberOfConesOnEachSideForEachConfig(
            cones: np.ndarray,
            configs: np.ndarray,
            coneType: ConeTypes,
            maxDistance: float,
            maxAngle: float,
        ) -> tuple[np.ndarray, np.ndarray]:
            Calculates the number of cones on each side for each configuration.

    Attributes:
        cachesCache: deque[
            tuple[tuple[int, ConeTypes], tuple[dict, dict, FloatArray, FloatArray]]
        ]:
            A deque that stores the cached values for different cone configurations.
    """

    def __init__(self) -> None:
        self.cachesCache: deque[
            tuple[
                tuple[int, ConeTypes],
                tuple[SearchDirectionsCacheType, AngleMaskCacheType, FloatArray, FloatArray],
            ]
        ] = deque(maxlen=20)

    def getCaches(
        self, cones: FloatArray, coneType: ConeTypes
    ) -> tuple[SearchDirectionsCacheType, AngleMaskCacheType, FloatArray, FloatArray,]:
        """
        Retrieves the cached values for the given cones and cone type.

        Args:
            cones: An array of cone coordinates.
            coneType: The type of cones.

        Returns:
            A tuple containing the cached values for search directions, angle cache,
            distance matrix square, and cones to cones.

        Raises:
            StopIteration: If the cached values for the given cones and cone type are not found.
        """
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
        cones: FloatArray,
        configs: IntArray,
        coneType: ConeTypes,
        maxDistance: float,
        maxAngle: float,
    ) -> tuple[IntArray, IntArray]:
        """
        Calculates the number of cones on each side for each configuration.

        Args:
            cones: An array of cone coordinates.
            configs: An array of configurations.
            coneType: The type of cones.
            maxDistance: The maximum distance for cone search.
            maxAngle: The maximum angle for cone search.

        Returns:
            A tuple containing the number of cones on each side for each configuration.

        Raises:
            StopIteration: If the cached values for the given cones and cone type are not found.
        """
        cachedValues = self.getCaches(cones, coneType)
        coneinfo = (cones, coneType)
        searchParameters = (maxDistance, maxAngle)
        return _implNumberOfConesOnEachSideForEachConfig(
            coneinfo, configs, searchParameters, *cachedValues[:]
        )


NEARBY_CONE_SEARCH = NearbyConeSearcher()


def _implNumberOfConesOnEachSideForEachConfig(  # pylint: disable=too-many-locals
    coneinfo: Tuple[FloatArray, ConeTypes],
    configs: IntArray,
    searchParameters: Tuple[float, float],
    existingSearchDirectionsCache: Optional[SearchDirectionsCacheType] = None,
    existingAngleMaskCache: Optional[AngleMaskCacheType] = None,
    distanceMatrixSquare: Optional[FloatArray] = None,
    conesToConesVecs: Optional[FloatArray] = None,
) -> tuple[IntArray, IntArray]:
    """
    For each configuration, find the number od cones that are on the expected side of
    the track, and the number of cones that are on the wrong side of the track.

    Args:
        cones = coneinfo[0]: array of cone positions and types
        configs: array of sorted cone configurations
        coneType = coneinfo[1]: type of cone to consider
        searchDistance = searchParameters[0]: the distance to search for cones
        searchAngle = searchParameters[1]: the angle to search for cones

    Returns:
        A tuple of two arrays, the first is the number of cones on the correct side of
        the track, the second is the number of cones on the wrong side of the track
    """

    conesXY = coneinfo[0][:, :2]

    idxsInAllConfigs = np.unique(configs)
    idxsInAllConfigs = idxsInAllConfigs[idxsInAllConfigs != -1]

    if conesToConesVecs is None:
        conesToConesVecs = conesXY - np.expand_dims(conesXY, 1)

    if distanceMatrixSquare is None:
        distanceMatrixSquare = myCdistSqEuclidean(conesXY, conesXY)
        np.fill_diagonal(distanceMatrixSquare, 1e6)

    nBadConesForAll: IntArray = np.zeros(len(configs), dtype=np.int_)
    nGoodConesForAll: IntArray = np.zeros(len(configs), dtype=np.int_)

    if existingAngleMaskCache is None:
        angleCache = createAngleCache()
    else:
        angleCache = existingAngleMaskCache

    for i, config in enumerate(configs):
        config = config[config != -1]

        for j, cone in enumerate(config):
            if j == 0:
                key = (config[j], SENTINEL_VALUE, config[j + 1])
            elif j == len(config) - 1:
                key = (config[j - 1], SENTINEL_VALUE, config[j])
            else:
                key = (config[j - 1], config[j], config[j + 1])

            maskGood, maskBad = calculateVisibleConesForOneCone(
                cone,
                (
                    distanceMatrixSquare < searchParameters[0] * searchParameters[0]
                ),  # DistanceMatrixMask
                key,
                conesToConesVecs,
                searchParameters[1],
                preCalculateSearchDirections(
                    coneinfo[0], configs, coneinfo[1], existingSearchDirectionsCache
                ),  # SearchDirectionCache
                angleCache,
                idxsToCheck=np.concatenate(
                    (
                        findNearbyConesForIdxs(
                            idxsInAllConfigs, distanceMatrixSquare, searchParameters[0]
                        ),
                        sortedSetDiff(idxsInAllConfigs, config),
                    )
                ),
            )

            nGoodConesForAll[i] += maskGood.sum()
            nBadConesForAll[i] += maskBad.sum()

    return nGoodConesForAll, nBadConesForAll


def preCalculateSearchDirections(
    cones: FloatArray,
    configs: IntArray,
    coneType: ConeTypes,
    existingCache: Optional[SearchDirectionsCacheType] = None,
) -> SearchDirectionsCacheType:
    """
    Pre-calculates search directions for nearby cones based on the given cone configurations.

    Args:
        cones (FloatArray): The array of cones.
        configs (IntArray): The array of cone configurations.
        coneType (ConeTypes): The type of cone.
        existingCache (Optional[SearchDirectionsCacheType], optional):
            The existing cache of search directions. Defaults to None.

    Returns:
        SearchDirectionsCacheType: The cache of pre-calculated search directions.
    """
    conesXY = cones[:, :2]
    if existingCache is not None:
        cache = existingCache
    else:
        cache = createSearchDirectionsCache()

    for config in configs:
        config = config[config != -1]
        assert len(config) >= 2

        keyFirst = (config[0], SENTINEL_VALUE, config[1])
        keyLast = (config[-2], SENTINEL_VALUE, config[-1])

        calculateMatchSearchDirectionForOneIfNotInCache(conesXY, keyFirst, coneType, cache)
        calculateMatchSearchDirectionForOneIfNotInCache(conesXY, keyLast, coneType, cache)

        for j in range(1, len(config) - 1):
            key = (config[j - 1], config[j], config[j + 1])
            calculateMatchSearchDirectionForOneIfNotInCache(conesXY, key, coneType, cache)

    return cache


def calculateMatchSearchDirectionForOneIfNotInCache(
    conesXY: FloatArray,
    key: SearchDirectionsCacheKeyType,
    coneType: ConeTypes,
    cacheDict: SearchDirectionsCacheType,
) -> FloatArray:
    """
    Calculates the search direction for a single cone if it is not already present in the cache.

    Args:
        conesXY (FloatArray): The XY coordinates of the cones.
        key (SearchDirectionsCacheKeyType): The key used to identify the search
         direction in the cache.
        coneType (ConeTypes): The type of cone.
        cacheDict (SearchDirectionsCacheType): The cache dictionary containing the
         search directions.

    Returns:
        FloatArray: The calculated search direction for the cone.

    """
    if key not in cacheDict:
        cacheDict[key] = calculateSearchDirectionForOne(conesXY, np.array(key[0::2]), coneType)

    return cacheDict[key]


def findNearbyConesForIdxs(
    idxs: IntArray, distanceMatrixSquare: FloatArray, searchRange: float
) -> IntArray:
    """
    Finds nearby cones for the given indices.

    Args:
        idxs (IntArray): The array of indices.
        distanceMatrixSquare (FloatArray): The square distance matrix.
        searchRange (float): The search range.

    Returns:
        IntArray: The array of nearby cone indices.
    """
    mask = distanceMatrixSquare[idxs] < searchRange * searchRange
    allIdxs = np.unique(mask.nonzero()[1])

    # calculate the set difference between allIdxs and idxs
    return sortedSetDiff(allIdxs, idxs)


def sortedSetDiff(arrayA: IntArray, arrayB: IntArray) -> IntArray:
    """Returns the set diffrence between a and b, assume a,b are sorted"""
    # we cannot use np.setdiff1d because it is not supported in numba
    mask: BoolArray = np.ones(len(arrayA), dtype=np.bool_)
    indices = np.clip(np.searchsorted(arrayA, arrayB), a_min=0, a_max=len(arrayA) - 1)
    mask[indices] = False
    return np.array(arrayA[mask]).astype(np.int_)


def calculateVisibleConesForOneCone(  # pylint: disable=too-many-arguments
    coneIdx: int,
    coneWithinDistanceMatrixMask: BoolArray,
    searchDirectionKey: SearchDirectionsCacheKeyType,
    conesToConesVecs: FloatArray,
    searchAngle: float,
    searchDirectionCache: SearchDirectionsCacheType,
    anglesBetweenSearchDirectionAndOtherConeCache: AngleMaskCacheType,
    idxsToCheck: IntArray,
) -> tuple[BoolArray, BoolArray]:
    """
    Calculates the visible cones for a given cone.

    Args:
        coneIdx (int): The index of the cone.
        coneWithinDistanceMatrixMask (BoolArray): The mask indicating which cones are
         within distance.
        searchDirectionKey (SearchDirectionsCacheKeyType): The key for the search direction cache.
        conesToConesVecs (FloatArray): The vectors from cones to cones.
        searchAngle (float): The search angle.
        searchDirectionCache (SearchDirectionsCacheType): The cache for search directions.
        anglesBetweenSearchDirectionAndOtherConeCache (AngleMaskCacheType):
            The cache for angles between search direction and other cones.
        idxsToCheck: The indices to check.

    Returns:
        tuple[BoolArray, BoolArray]: A tuple containing the good mask and the bad mask.
    """
    angleGoodMask: BoolArray = np.zeros(len(idxsToCheck), dtype=np.bool_)
    angleBadMask: BoolArray = np.zeros(len(idxsToCheck), dtype=np.bool_)
    for i, idx in enumerate(idxsToCheck):

        if not coneWithinDistanceMatrixMask[coneIdx, idx]:
            continue

        (
            angleGoodMask[i],
            angleBadMask[i],
        ) = angleBetweenSearchDirectionOfConeAndOtherConeIsTooLargeIfNotInCache(
            conesToConesVecs,
            searchDirectionKey,
            coneIdx,
            idx,
            searchDirectionCache,
            anglesBetweenSearchDirectionAndOtherConeCache,
            searchAngle,
        )

    maskDistance = coneWithinDistanceMatrixMask[coneIdx]

    goodMask = angleGoodMask & maskDistance[idxsToCheck]
    badMask = angleBadMask & maskDistance[idxsToCheck]

    return goodMask, badMask


def angleBetweenSearchDirectionOfConeAndOtherConeIsTooLargeIfNotInCache(
    allConeDirections: FloatArray,
    directionsKey: SearchDirectionsCacheKeyType,
    coneIdx: int,
    otherConeIdx: int,
    searchDirectionCache: SearchDirectionsCacheType,
    angleCache: AngleMaskCacheType,
    searchAngle: float,
) -> Tuple[bool, bool]:
    """
    Checks if the angle between the search direction of a cone and another cone is too large,
    if the angle is not already present in the cache.

    Args:
        allConeDirections (FloatArray): Array of all cone directions.
        directionsKey (SearchDirectionsCacheKeyType): Key for the search directions cache.
        coneIdx (int): Index of the cone.
        otherConeIdx (int): Index of the other cone.
        searchDirectionCache (SearchDirectionsCacheType): Cache for search directions.
        angleCache (AngleMaskCacheType): Cache for angle masks.
        searchAngle (float): Search angle.

    Returns:
        bool: True if the angle between the search direction of the cone and the other cone is
              too large, False otherwise.
    """
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
    directionsKey: SearchDirectionsCacheKeyType,
    coneIdx: int,
    otherConeIdx: int,
    searchDirectionCache: SearchDirectionsCacheType,
    searchAngle: float,
) -> tuple[bool, bool]:
    """
    Determines if the angle between the search direction of a cone and another cone is too large.

    Args:
        allConeDirections (FloatArray): Array of cone directions.
        directionsKey (SearchDirectionsCacheKeyType): Key for accessing search directions cache.
        coneIdx (int): Index of the cone.
        otherConeIdx (int): Index of the other cone.
        searchDirectionCache (SearchDirectionsCacheType): Cache of search directions.
        searchAngle (float): Search angle threshold.

    Returns:
        tuple[bool, bool]: A tuple containing two boolean values - `goodAngle` and `badAngle`.
            - `goodAngle`: True if the angle between the search direction and the other cone is
                within the threshold, False otherwise.
            - `badAngle`: True if the angle between the opposite search direction and the other
                cone is within the threshold, False otherwise.
    """
    fromConeToOtherCone = allConeDirections[coneIdx, otherConeIdx]
    searchDirection = searchDirectionCache[directionsKey]
    if np.isnan(searchDirection).any():
        searchDirection = np.array([0, 0])
    goodAngle = (
        vecAngleBetween(fromConeToOtherCone + 0.000001, searchDirection + 0.000001)
        < searchAngle / 2
    )
    badAngle = (
        vecAngleBetween(fromConeToOtherCone + 0.000001, -searchDirection + 0.000001)
        < searchAngle / 2
    )

    return bool(np.any(goodAngle)), bool(np.any(badAngle))


def numberConesOnEachSideForEachConfig(
    cones: FloatArray,
    configs: IntArray,
    coneType: ConeTypes,
    maxDistance: float,
    maxAngle: float,
) -> tuple[IntArray, IntArray]:
    """
    Counts the number of cones on each side for each configuration.

    Args:
        cones (np.ndarray): Array of cones.
        configs (np.ndarray): Array of configurations.
        coneType (ConeTypes): Type of cone.
        maxDistance (float): Maximum distance.
        maxAngle (float): Maximum angle.

    Returns:
        tuple[np.ndarray, np.ndarray]: A tuple containing two arrays:
            - Array of the number of cones on each side for each configuration.
            - Array of the number of cones on each side for each configuration.

    """
    return NEARBY_CONE_SEARCH.numberOfConesOnEachSideForEachConfig(
        cones, configs, coneType, maxDistance, maxAngle
    )


def createSearchDirectionsCache() -> SearchDirectionsCacheType:
    """
    Creates a cache of search directions for nearby cone search.

    Returns:
        SearchDirectionsCacheType: A dictionary containing search directions as keys
            and corresponding arrays as values.
    """
    return {(SENTINEL_VALUE, SENTINEL_VALUE, SENTINEL_VALUE): np.array([-1.0, -1.0])}


def createAngleCache() -> AngleMaskCacheType:
    """
    Creates and returns an angle cache dictionary.

    The angle cache dictionary stores information about cone angles for efficient cone sorting.
    The key of the dictionary is a tuple consisting of:
    - The key of the search direction cache (previous cone, current cone, next cone)
    - The index of the cone being considered
    - The index of the cone being compared to

    The value of the dictionary is a tuple of two booleans indicating whether
    the angles have been calculated.

    Returns:
    - The angle cache dictionary.

    Example:
    {
        (
            (SENTINEL_VALUE, SENTINEL_VALUE, SENTINEL_VALUE),
            SENTINEL_VALUE,
            SENTINEL_VALUE,
        ): (False, False),
    }
    """
    dicti = {
        (
            (SENTINEL_VALUE, SENTINEL_VALUE, SENTINEL_VALUE),
            SENTINEL_VALUE,
            SENTINEL_VALUE,
        ): (False, False),
    }

    return dicti
