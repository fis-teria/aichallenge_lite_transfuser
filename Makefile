.PHONY: install test smoke demo clean

install:
	python -m pip install -e .[dev]

test:
	pytest -q

smoke:
	python tools/smoke_test.py --config configs/transfuser_lite_v0.yaml

demo:
	python tools/build_demo_dataset.py --output /tmp/aic_demo --samples 64
	python -m aic_transfuser_lite.training.train --config configs/transfuser_lite_v0.yaml --train-index /tmp/aic_demo/index.csv --val-index /tmp/aic_demo/index.csv --output /tmp/aic_run --epochs 1

clean:
	rm -rf .pytest_cache build dist *.egg-info /tmp/aic_demo /tmp/aic_run
