import cv2
import pytesseract
import numpy as np
import numpy.typing as npt
from collections import Counter
from typing import Sequence, cast
from typing_extensions import override

from .ocr_config import OCRConfig
from ...core.module import DogModule


class OCRModule(DogModule):
    """
    Users should not access or construct this class directly.
    Rather, they should access it through the :class:`~core.controller.Go2Controller` instance.
    """

    def __init__(self) -> None:
        super().__init__("OCR")


    @override
    def _initialize(self) -> None:
        if self._initialized:
            return

        self._initialized = True

    # See ref: https://medium.com/@EnginDenizTangut/from-image-to-voice-building-an-ocr-tts-app-with-python-opencv-tesseract-5f5db8ea3b7b
    def extract_text_from_image(self, image: npt.NDArray[np.uint8], config: OCRConfig = OCRConfig()) -> tuple[list[str], npt.NDArray[np.uint8]]:
        """
        Run OCR on a single image and return the detected text along with an annotated copy.

        Parameters
        ----------
        image : np.ndarray
            Input BGR image to extract text from.
        config : OCRConfig, optional
            OCR configuration controlling preprocessing, confidence threshold, and Tesseract CLI options.

        Returns
        -------
        tuple[list[str], np.ndarray]
            The list of confidently detected words and the annotated image with bounding boxes drawn around them.
        """
        words, annotated = self._filter_and_highlight(self._preprocess(image), config)
        return words, annotated
    

    def extract_text_from_images_temporal_voting(self, images: Sequence[npt.NDArray[np.uint8]], config: OCRConfig = OCRConfig())-> tuple[list[str], list[npt.NDArray[np.uint8]]]:
        """
        Run OCR across multiple images of the same scene and keep only words detected
        in at least ``config.temporal_voting_threshold`` of them, reducing false positives
        from single-frame noise.

        Parameters
        ----------
        images : np.ndarray[np.ndarray]
            Array of input BGR images to extract text from.
        config : OCRConfig, optional
            OCR configuration controlling preprocessing, confidence threshold, temporal voting threshold, and Tesseract CLI options.

        Returns
        -------
        tuple[list[str], np.ndarray[np.ndarray]]
            The list of words that met the temporal voting threshold and the array of annotated images corresponding to each input image.

        Raises
        ------
        ValueError
            If the number of provided images is less than ``config.temporal_voting_threshold``.
        """
        if len(images) < config.temporal_voting_threshold:
            raise ValueError(
                f"The number of provided images ({len(images)}) is less than the required "
                f"temporal voting threshold ({config.temporal_voting_threshold})."
            )
        
        annotated_images = np.empty(len(images), dtype=object)
        detected_words = []

        for idx, image in enumerate(images):
            words, annotated = self._filter_and_highlight(self._preprocess(image), config)
            detected_words.extend(words)
            annotated_images[idx] = annotated

        word_counts = Counter(detected_words)
        final_words = [word for word, count in word_counts.items() if count >= config.temporal_voting_threshold]
        return final_words, cast(list[npt.NDArray[np.uint8]], annotated_images)
    

    def _preprocess(self, image: npt.NDArray[np.uint8]) -> npt.NDArray[np.uint8]:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray, None, fx=1.5, fy=1.5, interpolation=cv2.INTER_CUBIC)
        gray = cv2.bilateralFilter(gray, 9, 75, 75)

        thresh = cv2.adaptiveThreshold(
            gray, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, 31, 10
        )

        kernel = np.ones((2, 2), np.uint8)
        clean = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        clean = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, kernel)
        return cast(npt.NDArray[np.uint8], clean)


    def _filter_and_highlight(self, image: npt.NDArray[np.uint8], config: OCRConfig) -> tuple[list[str], npt.NDArray[np.uint8]]:
        ocr_data = pytesseract.image_to_data(image, output_type=pytesseract.Output.DICT, config=config.cli_command)
        output = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        confident_words = []
        for i in range (len(ocr_data["text"])):
            word = ocr_data['text'][i].strip()
            conf = int(ocr_data['conf'][i])
            
            if conf > config.min_conf:
                confident_words.append(word)
                x, y, w, h = (ocr_data["left"][i], ocr_data['top'][i], ocr_data['width'][i], ocr_data['height'][i])
                cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 2)

        return confident_words, cast(npt.NDArray[np.uint8], output)


    @override
    def _shutdown(self) -> None:
        self._initialized = False