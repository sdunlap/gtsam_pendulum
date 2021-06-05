
# Log 
## 4 June

Custom factors for Python have been implemented by ProfFan but have not been merged into the main ```develop``` branch. A pull request was issued about a month ago, but dellaert wisely required more documentation and examples before pulling it into the main branch. The pull request thread went quiet 10 days ago.

Currently, the feature can only be accessed through this branch: ```feature/custom_factor```

I spent 10 hours trying to get optimization working. Originally, I thought I was doing something wrong with my derivatives, so I spent a long time trying various things. In the end, creating the custom factors is pretty straight forward, but there is a low-level bug that causes all ```optimize()``` calls to hang indefinietly. All preexisting unit tests pass, so the problem must be at a lower level that ProfFan has not tested yet. He should not have tried to merge this into ```develop``` without testing optimization first.

## 5 June

At 7am I figured out a workaround (disabling TBB when compiling gtsam). This was mostly luck as ProfFan mentioned briefly in one of his docs that the custom factors won't work with parallel evaluation. TBB is the first thing I found related to parallelization that I could disable.

I found out that at 2am on 5 June, ProfFan pushed a change that fixed the problem I was dealing with. He also added an actual optimization example. His original code only tested the CustomFactor class by comparing it directly to other Factor types. The bug only came up when running a graph optimizer. My guess is he discovered the bug while implementing the optimization example dellaert requested. His note:
```
ProfFan commented 5 hours ago

Changes:

* Per @dellaert 's request, I added the example trajectory optimization with CustomFactor

* Unit test of optimizing an actual graph

* Fixed linearization of CustomFactor with TBB. Now it will not deadlock trying to acquire the GIL
```

Custom factors are now working, and I have two functional examples besides the one that ProfFan added: 

* LocalizationExample.py - This is a simple example based on ```LocalizationExample.cpp```. According to the table in ```gtsam/examples/README.md```, this example was believed to be impossible in python before now.

* UnicycleExample.py - This is a slightly modified version of the example developed by cntaylor. ProfFan's initial examples were duplicates of the BetweenFactor, so modifying this example was a good first step. I replaced the prior factor and the between factors with custom ones. 



### Passing measurements to the custom factor

I'm aware of two methods for passing the measurements to the custom factor. 

* ProfFan's method - LocalizationExample.py - This example uses the partial(...) method for passing measurements to the error function.
``` py
def error_func(this: gtsam.CustomFactor, v: gtsam.Values, H: Optional[List[np.ndarray]], mx, my):
	q = v.atPose2(this.keys()[0])
	...

graph.add(gtsam.CustomFactor(UNARY_NOISE, gtsam.KeyVector([1]), partial(error_func, mx=0.0, my=0.0)))

```

My method - UnicycleExample.py - This example uses a python class to store and access the measurements. I'm not entirely sure how scope works for the error functions, but it appears this method works and may be easier to maintain for more compicated systems.

``` py
@dataclass
class MyCustomPriorFactor:
	
	expected : gtsam.Pose2

	def error_func(self, this: gtsam.CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
		# Get the variable values from `v`
		key0 = this.keys()[0]
		# Calculate non-linear error
		x = v.atPose2(key0)
		error = self.expected.localCoordinates(x)
        ...

mf = MeasFactor.MyCustomPriorFactor(x0)
graph.add(gtsam.CustomFactor(prior_noise, [pose_key(0)], mf.error_func))

```


# References

https://github.com/borglab/gtsam/pull/767

https://github.com/borglab/gtsam/issues/748


https://github.com/borglab/gtsam/blob/develop/python/gtsam/examples/README.md

https://github.com/borglab/gtsam/blob/feature/custom_factor/python/CustomFactors.md
https://github.com/cntaylor/gtsam-example

https://github.com/cntaylor/gtsam-python-docker-vscode