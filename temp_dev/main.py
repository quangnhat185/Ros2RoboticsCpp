from typing import List

class Solution:
    def searchInsert(self, nums: List[int], target: int) -> int:
        
        for i in range(0, len(nums)):
            if nums[i] == target: return i
            elif target < nums[i]: return i
            elif target > nums[-1]: return (len(nums))
        

if __name__ == "__main__":

    solution = Solution()
    nums = [1,3,5,6]
    target = 7

    print(solution.searchInsert(nums, target))