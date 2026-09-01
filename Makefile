.PHONY: test-v3 bag-scan dataset-audit

test-v3:
	python -m pytest -q

bag-scan:
	python -m aic_transfuser_lite.cli bag scan --input-root "$(INPUT)" --output "$(OUTPUT)"

dataset-audit:
	python -m aic_transfuser_lite.cli dataset audit --dataset-root "$(DATASET)" --output "$(OUTPUT)"
