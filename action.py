class Action(object):
      """Actions processed by the Controller to make drinks."""

        def __call__(self, robot):
                raise NotImplementedError()


# Example actions:
# Hunt for Bottle
# Position Over Bottle
# Lower On Bottle
# Grab Bottle
# Pick up Bottle
# Dispense
# Lower bottle
# Replace bottle
