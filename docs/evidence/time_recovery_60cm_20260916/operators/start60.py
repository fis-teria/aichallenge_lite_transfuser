"""Start one explicitly planned three-event AWSIM lap."""
import argparse
import ops60

parser=argparse.ArgumentParser()
parser.add_argument('--run',required=True)
args=parser.parse_args()
ops60.start(args.run)
